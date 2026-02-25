#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Scenario Orchestrator for Gazebo (ROS1 / Gazebo Classic)

Merges the responsibilities of:
- migbot_apf_controller/scripts/gazebo_scenario.py  (publish /scenario/input_* from /gazebo/model_states,
  and set model poses via /gazebo/set_model_state)
- migbot_apf_controller/scripts/scenario_manager.py (automatic scenario switching on collision/goal)

Key features:
1) Positions robot + obstacles for a selected scenario using /gazebo/set_model_state
2) Moves ALL non-participating obstacles to a "parking lot" far away (unique spots) with zero velocity
3) Publishes:
   - /scenario/input_robot      (dynamic_obstacle_avoidance/RobotState)
   - /scenario/input_obstacles  (dynamic_obstacle_avoidance/ObstacleArray)
   - /current_scenario          (std_msgs/String)
4) Automatically advances to the next scenario when:
   - /obstacle_avoidance/collision == True
   - OR /apfm/reached_goal == True
   - OR /obstacle_avoidance/distance_to_goal <= goal_tolerance
5) Allows forcing a scenario via topic:
   - /scenario_orchestrator/set_scenario (std_msgs/Int32)  -> scenario id (1..14)

Scenario definitions (requested):
  vegetation3_buoy scenarios 1..7
  trunk1_buoy      scenarios 8..14 (same motion patterns)

Default sequence begins with HEAD-ON, then CROSSING, then OVERTAKING.
"""

import math
import threading
from typing import Dict, List, Tuple, Set

import rospy
import numpy as np
from geometry_msgs.msg import Quaternion
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Bool, Float64, Int32, String
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler

from dynamic_obstacle_avoidance.msg import RobotState, ObstacleState, ObstacleArray


ROBOT_NAME = "migbot"

OBSTACLE_NAMES = [
    "vegetation1_buoy","vegetation3_buoy","branche3_buoy",
    "vegetation3_buoy_clone","vegetation3_buoy_clone_clone","vegetation3_buoy_clone_clone_clone",
    "branche3_buoy_clone_clone_clone_clone","trunk1_buoy","trunk1_buoy_clone","branche1_buoy"
]

OBSTACLE_RADIUS = [1, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 3.15, 3.15, 4.4]

VEGETATION_SCENARIOS = {
    1: {"vegetation3_buoy": {"position": [44, 44], "velocity": [0, 0]}},
    2: {"vegetation3_buoy": {"position": [60, 60], "velocity": [-0.85, -0.85]}},   # head-on
    3: {"vegetation3_buoy": {"position": [50, 0],  "velocity": [0, 0.85]}},        # crossing
    4: {"vegetation3_buoy": {"position": [50, 0],  "velocity": [-0.2, 0.95]}},     # crossing (maneuver)
    5: {"vegetation3_buoy": {"position": [0, 50],  "velocity": [0.85, 0]}},
    6: {"vegetation3_buoy": {"position": [0, 50],  "velocity": [0.7, -0.2]}},
    7: {"vegetation3_buoy": {"position": [-10, -10], "velocity": [1.5, 1.5]}},     # overtaking-style
}

TRUNK_SCENARIOS = {
    8: {"trunk1_buoy": {"position": [44, 44], "velocity": [0, 0]}},
    9: {"trunk1_buoy": {"position": [60, 60], "velocity": [-0.85, -0.85]}},        # head-on
    10: {"trunk1_buoy": {"position": [50, 0], "velocity": [0, 0.85]}},             # crossing
    11: {"trunk1_buoy": {"position": [50, 0], "velocity": [-0.2, 0.95]}},          # crossing (maneuver)
    12: {"trunk1_buoy": {"position": [0, 50], "velocity": [0.85, 0]}},
    13: {"trunk1_buoy": {"position": [0, 50], "velocity": [0.7, -0.2]}},
    14: {"trunk1_buoy": {"position": [-10, -10], "velocity": [1.5, 1.5]}},         # overtaking-style
}

SCENARIOS: Dict[int, Dict[str, Dict[str, List[float]]]] = {}
SCENARIOS.update(VEGETATION_SCENARIOS)
SCENARIOS.update(TRUNK_SCENARIOS)

HEAD_ON = [2, 9]
CROSSING = [3, 4, 5, 6, 10, 11, 12, 13]
OVERTAKING = [7, 14]
DEFAULT_SEQUENCE = HEAD_ON + CROSSING + OVERTAKING


def quat_from_yaw_deg(yaw_deg: float) -> Quaternion:
    yaw_rad = math.radians(yaw_deg)
    q = quaternion_from_euler(0.0, 0.0, yaw_rad)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def make_model_state(name: str,
                     pos_xy: Tuple[float, float],
                     vel_xy: Tuple[float, float],
                     yaw_deg: float = 0.0) -> ModelState:
    st = ModelState()
    st.model_name = name
    st.reference_frame = "world"

    st.pose.position.x = float(pos_xy[0])
    st.pose.position.y = float(pos_xy[1])
    st.pose.position.z = 0.0
    st.pose.orientation = quat_from_yaw_deg(yaw_deg)

    st.twist.linear.x = float(vel_xy[0])
    st.twist.linear.y = float(vel_xy[1])
    st.twist.linear.z = 0.0

    # Clear angular velocities to "zero physics"
    st.twist.angular.x = 0.0
    st.twist.angular.y = 0.0
    st.twist.angular.z = 0.0
    return st


class ScenarioOrchestrator:
    def __init__(self):
        rospy.init_node("scenario_orchestrator", anonymous=False)

        # Params
        self.robot_initial_pose = np.array(rospy.get_param("~robot_initial_pose", [0.0, 0.0]), dtype=float)
        self.robot_initial_vel = np.array(rospy.get_param("~robot_initial_velocity", [0.0, 0.0]), dtype=float)
        self.robot_initial_yaw_deg = float(rospy.get_param("~robot_initial_yaw_deg", 45.0))

        self.obstacle_yaw_deg = float(rospy.get_param("~obstacle_yaw_deg", 0.0))

        self.park_base = np.array(rospy.get_param("~park_base_xy", [500.0, 500.0]), dtype=float)
        self.park_spacing = float(rospy.get_param("~park_spacing", 10.0))

        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 1.0))
        self.cooldown_s = float(rospy.get_param("~transition_cooldown_s", 2.0))

        self.pause_during_reset = bool(rospy.get_param("~pause_physics_during_reset", True))
        self.call_reset_simulation = bool(rospy.get_param("~call_reset_simulation", True))

        seq = rospy.get_param("~scenario_sequence", DEFAULT_SEQUENCE)
        self.sequence: List[int] = [int(x) for x in seq] if seq else DEFAULT_SEQUENCE.copy()
        if not self.sequence:
            self.sequence = DEFAULT_SEQUENCE.copy()

        self.seq_index = 0
        self.current_scenario_id = self.sequence[self.seq_index]

        self._lock = threading.Lock()
        self._transitioning = False
        self._last_transition_time = rospy.Time(0)

        # ModelStates knowledge
        self._known_models: Set[str] = set()
        self._have_model_states = threading.Event()

        # Services
        rospy.wait_for_service("/gazebo/set_model_state")
        self._set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        self._pause = None
        self._unpause = None
        self._reset_sim = None
        try:
            rospy.wait_for_service("/gazebo/pause_physics", timeout=2.0)
            rospy.wait_for_service("/gazebo/unpause_physics", timeout=2.0)
            self._pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
            self._unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        except Exception:
            rospy.logwarn("[scenario_orchestrator] pause/unpause services not available")

        try:
            rospy.wait_for_service("/gazebo/reset_simulation", timeout=2.0)
            self._reset_sim = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)
        except Exception:
            rospy.logwarn("[scenario_orchestrator] reset_simulation service not available")

        # Publishers
        self.robot_pub = rospy.Publisher("/scenario/input_robot", RobotState, queue_size=10)
        self.obstacles_pub = rospy.Publisher("/scenario/input_obstacles", ObstacleArray, queue_size=10)
        self.current_scenario_pub = rospy.Publisher("/current_scenario", String, queue_size=10)

        # State holders
        self._migbot_state = RobotState()
        self._obstacles = {name: ObstacleState(name=name, radius=r) for name, r in zip(OBSTACLE_NAMES, OBSTACLE_RADIUS)}
        self._obstacle_present = {name: False for name in OBSTACLE_NAMES}

        # Subscribers
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_cb, queue_size=1)

        rospy.Subscriber("/obstacle_avoidance/collision", Bool, self._collision_cb, queue_size=10)
        rospy.Subscriber("/scenario/reached_goal", Bool, self._reached_goal_cb, queue_size=10)
        rospy.Subscriber("/obstacle_avoidance/distance_to_goal", Float64, self._distance_to_goal_cb, queue_size=10)

        rospy.Subscriber("/scenario_orchestrator/set_scenario", Int32, self._set_scenario_cb, queue_size=10)

        # Wait briefly for first ModelStates so we know which models exist (avoids spam service calls)
        self._wait_first_model_states()

        # Apply first scenario
        self.apply_scenario(self.current_scenario_id, reason="startup")
        rospy.loginfo(f"[scenario_orchestrator] Started at scenario {self.current_scenario_id}")

    def _wait_first_model_states(self):
        # up to 5 seconds
        if self._have_model_states.wait(timeout=5.0):
            return
        rospy.logwarn("[scenario_orchestrator] No /gazebo/model_states received yet; will still try to set states.")

    # -------- ModelStates -> /scenario/input_* --------

    def _model_states_cb(self, msg: ModelStates):
        self._known_models = set(msg.name)
        self._have_model_states.set()

        # Reset "present" flags
        for n in OBSTACLE_NAMES:
            self._obstacle_present[n] = False

        # Update robot + obstacles
        for i, name in enumerate(msg.name):
            if name == ROBOT_NAME:
                self._migbot_state.position = msg.pose[i].position
                self._migbot_state.velocity = msg.twist[i].linear
                self._migbot_state.orientation = msg.pose[i].orientation
                self._migbot_state.radius = float(rospy.get_param("/apfm_obstacle_avoidance/robot_domain_radius", 1.4))
            elif name in self._obstacles:
                ob = self._obstacles[name]
                ob.position = msg.pose[i].position
                ob.velocity = msg.twist[i].linear
                self._obstacle_present[name] = True

        # Publish robot
        self.robot_pub.publish(self._migbot_state)

        # Publish only obstacles that exist in the world (mimics your old "found" behavior)
        arr = ObstacleArray()
        arr.obstacles = [self._obstacles[n] for n in OBSTACLE_NAMES if self._obstacle_present[n]]
        self.obstacles_pub.publish(arr)

    # -------- Transition triggers --------

    def _cooldown_ok(self) -> bool:
        return (rospy.Time.now() - self._last_transition_time).to_sec() >= self.cooldown_s

    def _collision_cb(self, msg: Bool):
        if msg.data:
            self._request_advance("collision")

    def _reached_goal_cb(self, msg: Bool):
        if msg.data:
            self._request_advance("reached_goal_topic")

    def _distance_to_goal_cb(self, msg: Float64):
        if msg.data <= self.goal_tolerance:
            self._request_advance(f"distance_to_goal<=tol ({msg.data:.3f} <= {self.goal_tolerance})")

    def _set_scenario_cb(self, msg: Int32):
        scenario_id = int(msg.data)
        if scenario_id not in SCENARIOS:
            rospy.logwarn(f"[scenario_orchestrator] Unknown scenario id={scenario_id}. Known: {sorted(SCENARIOS.keys())}")
            return

        with self._lock:
            self.current_scenario_id = scenario_id
            if scenario_id in self.sequence:
                self.seq_index = self.sequence.index(scenario_id)
        self.apply_scenario(scenario_id, reason="manual_set")

    def _request_advance(self, reason: str):
        with self._lock:
            if self._transitioning or not self._cooldown_ok():
                return
            self._transitioning = True
        threading.Thread(target=self._advance_worker, args=(reason,), daemon=True).start()

    def _advance_worker(self, reason: str):
        try:
            with self._lock:
                self.seq_index = (self.seq_index + 1) % len(self.sequence)
                self.current_scenario_id = self.sequence[self.seq_index]
                next_id = self.current_scenario_id
            self.apply_scenario(next_id, reason=f"auto:{reason}")
        finally:
            with self._lock:
                self._transitioning = False
                self._last_transition_time = rospy.Time.now()

    # -------- Apply scenario --------

    def apply_scenario(self, scenario_id: int, reason: str = ""):
        if scenario_id not in SCENARIOS:
            rospy.logerr(f"[scenario_orchestrator] Scenario {scenario_id} not defined.")
            return

        scenario_cfg = SCENARIOS[scenario_id]
        active_names = set(scenario_cfg.keys())

        scenario_name = f"sceinario_dissertacao_{scenario_id}"
        self.current_scenario_pub.publish(String(data=scenario_name))
        rospy.set_param("/gazebo_scenario/scenario", scenario_name)  # compatibility with other tools/logs

        rospy.loginfo(f"[scenario_orchestrator] Applying {scenario_name} (reason={reason})")

        try:
            if self.pause_during_reset and self._pause is not None:
                self._pause()

            if self.call_reset_simulation and self._reset_sim is not None:
                self._reset_sim()
                rospy.sleep(0.2)

            # Robot
            self._set_if_exists(make_model_state(
                ROBOT_NAME,
                (float(self.robot_initial_pose[0]), float(self.robot_initial_pose[1])),
                (float(self.robot_initial_vel[0]), float(self.robot_initial_vel[1])),
                yaw_deg=self.robot_initial_yaw_deg
            ))

            # Obstacles
            park_positions = self._parking_positions()

            for idx, name in enumerate(OBSTACLE_NAMES):
                if name in active_names:
                    cfg = scenario_cfg[name]
                    pos = cfg["position"]
                    vel = cfg["velocity"]
                    self._set_if_exists(make_model_state(
                        name, (pos[0], pos[1]), (vel[0], vel[1]), yaw_deg=self.obstacle_yaw_deg
                    ))
                else:
                    park_xy = park_positions[idx]
                    self._set_if_exists(make_model_state(
                        name, (park_xy[0], park_xy[1]), (0.0, 0.0), yaw_deg=self.obstacle_yaw_deg
                    ))

            rospy.sleep(0.1)

        except rospy.ServiceException as e:
            rospy.logerr(f"[scenario_orchestrator] Gazebo service call failed: {e}")
        finally:
            if self.pause_during_reset and self._unpause is not None:
                try:
                    self._unpause()
                except Exception:
                    pass

    def _set_if_exists(self, state: ModelState):
        # If we already know the models present, avoid spamming set_model_state for non-existing ones.
        if self._known_models and state.model_name not in self._known_models:
            return
        resp = self._set_state_srv(state)
        if not resp.success:
            rospy.logwarn(f"[scenario_orchestrator] set_model_state failed for {state.model_name}: {resp.status_message}")

    def _parking_positions(self) -> List[Tuple[float, float]]:
        base_x, base_y = float(self.park_base[0]), float(self.park_base[1])
        positions: List[Tuple[float, float]] = []
        for i in range(len(OBSTACLE_NAMES)):
            row = i // 5
            col = i % 5
            positions.append((base_x + col * self.park_spacing, base_y + row * self.park_spacing))
        return positions


if __name__ == "__main__":
    try:
        ScenarioOrchestrator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
