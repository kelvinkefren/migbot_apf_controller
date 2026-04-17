#!/usr/bin/env python3

"""
Scenario manager/orchestrator for Gazebo that:
- positions the robot and scenario obstacles through /gazebo/set_model_state
- publishes only /scenario/input_robot and /current_scenario
- does NOT publish /scenario/input_obstacles

This is intended for pipelines where obstacle perception publishes
/scenario/input_obstacles independently (for example, lidar perception or
obstacle_detector_ros1).
"""

import math
import threading

import numpy as np
import rospy
from dynamic_obstacle_avoidance.msg import RobotState
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool, Float64, Int32, String
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler


ROBOT_NAME = "migbot"

OBSTACLE_NAMES = [
    "vegetation1_buoy",
    "vegetation3_buoy",
    "branche3_buoy",
    "vegetation3_buoy_clone",
    "vegetation3_buoy_clone_clone",
    "vegetation3_buoy_clone_clone_clone",
    "branche3_buoy_clone_clone_clone_clone",
    "trunk1_buoy",
    "trunk1_buoy_clone",
    "branche1_buoy",
]


def _make_scenario(group, kind, obstacles, events=None, description=""):
    return {
        "group": int(group),
        "kind": str(kind),
        "description": str(description),
        "obstacles": obstacles,
        "events": events or [],
    }


SCENARIOS = {
    1: _make_scenario(
        1,
        "head_on",
        {
            "vegetation3_buoy": {"position": [60.0, 60.0], "velocity": [-0.85, -0.85]},
        },
        description="Grupo 1: head-on elementar com um obstaculo.",
    ),
    2: _make_scenario(
        1,
        "crossing_a",
        {
            "vegetation3_buoy": {"position": [50.0, 0.0], "velocity": [0.0, 0.85]},
        },
        description="Grupo 1: crossing A elementar com um obstaculo.",
    ),
    3: _make_scenario(
        1,
        "crossing_b",
        {
            "vegetation3_buoy": {"position": [0.0, 50.0], "velocity": [0.85, 0.0]},
        },
        description="Grupo 1: crossing B elementar com um obstaculo.",
    ),
    4: _make_scenario(
        1,
        "overtaking",
        {
            "vegetation3_buoy": {"position": [-10.0, -10.0], "velocity": [1.50, 1.50]},
        },
        description="Grupo 1: overtaking elementar com um obstaculo.",
    ),
    5: _make_scenario(
        2,
        "head_on_speed_change",
        {
            "vegetation3_buoy": {"position": [60.0, 60.0], "velocity": [-0.85, -0.85]},
        },
        events=[
            {
                "time": 8.0,
                "description": "Aumento temporario da velocidade no encontro frontal.",
                "updates": {
                    "vegetation3_buoy": {"velocity": [-1.20, -1.20]},
                },
            }
        ],
        description="Grupo 2: head-on com perturbacao temporal de velocidade.",
    ),
    6: _make_scenario(
        2,
        "crossing_a_speed_change",
        {
            "vegetation3_buoy": {"position": [50.0, 0.0], "velocity": [0.0, 0.85]},
        },
        events=[
            {
                "time": 8.0,
                "description": "Mudanca combinada de direcao e velocidade em crossing A.",
                "updates": {
                    "vegetation3_buoy": {"velocity": [-0.20, 0.95]},
                },
            }
        ],
        description="Grupo 2: crossing A com mudanca temporal de direcao e velocidade.",
    ),
    7: _make_scenario(
        2,
        "crossing_b_speed_change",
        {
            "vegetation3_buoy": {"position": [0.0, 50.0], "velocity": [0.85, 0.0]},
        },
        events=[
            {
                "time": 8.0,
                "description": "Mudanca temporal de velocidade em crossing B.",
                "updates": {
                    "vegetation3_buoy": {"velocity": [0.70, -0.20]},
                },
            }
        ],
        description="Grupo 2: crossing B com perturbacao temporal.",
    ),
    8: _make_scenario(
        2,
        "overtaking_speed_change",
        {
            "vegetation3_buoy": {"position": [-10.0, -10.0], "velocity": [1.20, 1.20]},
        },
        events=[
            {
                "time": 8.0,
                "description": "Aceleracao com desvio leve durante overtaking.",
                "updates": {
                    "vegetation3_buoy": {"velocity": [1.60, 0.90]},
                },
            }
        ],
        description="Grupo 2: overtaking com perturbacao temporal.",
    ),
    9: _make_scenario(
        3,
        "multi_head_on_crossing",
        {
            "vegetation3_buoy": {"position": [60.0, 60.0], "velocity": [-0.85, -0.85]},
            "trunk1_buoy": {"position": [50.0, 0.0], "velocity": [0.0, 0.85]},
        },
        description="Grupo 3: encontro frontal simultaneo a um crossing.",
    ),
    10: _make_scenario(
        3,
        "dual_crossing_conflict",
        {
            "vegetation3_buoy": {"position": [50.0, 0.0], "velocity": [0.0, 0.85]},
            "trunk1_buoy": {"position": [0.0, 50.0], "velocity": [0.85, 0.0]},
            "vegetation1_buoy": {"position": [55.0, 20.0], "velocity": [-0.60, 0.50]},
        },
        description="Grupo 3: crossing multiplos com potencial conflito de decisao.",
    ),
    11: _make_scenario(
        3,
        "overtaking_with_oncoming",
        {
            "vegetation3_buoy": {"position": [-10.0, -10.0], "velocity": [1.50, 1.50]},
            "trunk1_buoy": {"position": [60.0, 60.0], "velocity": [-0.85, -0.85]},
            "branche3_buoy": {"position": [0.0, 50.0], "velocity": [0.85, 0.0]},
        },
        description="Grupo 3: overtaking combinado com frontal e crossing.",
    ),
    12: _make_scenario(
        3,
        "congested_corridor",
        {
            "vegetation3_buoy": {"position": [44.0, 44.0], "velocity": [0.0, 0.0]},
            "trunk1_buoy": {"position": [30.0, 58.0], "velocity": [0.35, -0.25]},
            "vegetation1_buoy": {"position": [58.0, 30.0], "velocity": [-0.25, 0.35]},
            "branche1_buoy": {"position": [25.0, 25.0], "velocity": [0.0, 0.0]},
        },
        description="Grupo 3: ambiente congestionado com obstaculos estaticos e dinamicos.",
    ),
    13: _make_scenario(
        4,
        "multi_head_on_crossing_event",
        {
            "vegetation3_buoy": {"position": [60.0, 60.0], "velocity": [-0.85, -0.85]},
            "trunk1_buoy": {"position": [50.0, 0.0], "velocity": [0.0, 0.85]},
        },
        events=[
            {
                "time": 8.0,
                "description": "Aproximacao frontal acelera enquanto crossing desvia.",
                "updates": {
                    "vegetation3_buoy": {"velocity": [-1.10, -1.10]},
                    "trunk1_buoy": {"velocity": [-0.20, 0.95]},
                },
            }
        ],
        description="Grupo 4: ambiente congestionado com evento em dois obstaculos.",
    ),
    14: _make_scenario(
        4,
        "dual_crossing_event",
        {
            "vegetation3_buoy": {"position": [50.0, 0.0], "velocity": [0.0, 0.85]},
            "trunk1_buoy": {"position": [0.0, 50.0], "velocity": [0.85, 0.0]},
            "vegetation1_buoy": {"position": [55.0, 20.0], "velocity": [-0.60, 0.50]},
        },
        events=[
            {
                "time": 8.0,
                "description": "Dois crossings mudam de velocidade em momentos de conflito.",
                "updates": {
                    "vegetation3_buoy": {"velocity": [-0.20, 0.95]},
                    "trunk1_buoy": {"velocity": [0.70, -0.20]},
                },
            }
        ],
        description="Grupo 4: crossing multiplos com eventos temporais.",
    ),
    15: _make_scenario(
        4,
        "overtaking_with_oncoming_event",
        {
            "vegetation3_buoy": {"position": [-10.0, -10.0], "velocity": [1.20, 1.20]},
            "trunk1_buoy": {"position": [60.0, 60.0], "velocity": [-0.85, -0.85]},
            "branche3_buoy": {"position": [0.0, 50.0], "velocity": [0.85, 0.0]},
        },
        events=[
            {
                "time": 8.0,
                "description": "Ultrapassagem acelera enquanto obstaculo lateral muda o rumo.",
                "updates": {
                    "vegetation3_buoy": {"velocity": [1.60, 0.90]},
                    "branche3_buoy": {"velocity": [0.70, -0.20]},
                },
            }
        ],
        description="Grupo 4: conflitos simultaneos com evento temporal.",
    ),
    16: _make_scenario(
        4,
        "congested_corridor_event",
        {
            "vegetation3_buoy": {"position": [44.0, 44.0], "velocity": [0.0, 0.0]},
            "trunk1_buoy": {"position": [30.0, 58.0], "velocity": [0.35, -0.25]},
            "vegetation1_buoy": {"position": [58.0, 30.0], "velocity": [-0.25, 0.35]},
            "branche1_buoy": {"position": [25.0, 25.0], "velocity": [0.0, 0.0]},
        },
        events=[
            {
                "time": 8.0,
                "description": "Dois obstaculos dinamicos aceleram no ambiente congestionado.",
                "updates": {
                    "trunk1_buoy": {"velocity": [0.65, -0.35]},
                    "vegetation1_buoy": {"velocity": [-0.35, 0.65]},
                },
            }
        ],
        description="Grupo 4: ambiente congestionado com eventos e replanejamento local.",
    ),
}

DEFAULT_SEQUENCE = sorted(SCENARIOS.keys())


def quat_from_yaw_deg(yaw_deg):
    q = quaternion_from_euler(0.0, 0.0, math.radians(float(yaw_deg)))
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def make_model_state(name, pos_xy, vel_xy, yaw_deg=0.0):
    state = ModelState()
    state.model_name = name
    state.reference_frame = "world"
    state.pose.position.x = float(pos_xy[0])
    state.pose.position.y = float(pos_xy[1])
    state.pose.position.z = 0.0
    state.pose.orientation = quat_from_yaw_deg(yaw_deg)
    state.twist.linear.x = float(vel_xy[0])
    state.twist.linear.y = float(vel_xy[1])
    state.twist.linear.z = 0.0
    state.twist.angular.x = 0.0
    state.twist.angular.y = 0.0
    state.twist.angular.z = 0.0
    return state


class GroupedScenarioManager:
    def __init__(self):
        rospy.init_node("grouped_scenario_manager", anonymous=False)

        self.robot_initial_pose = np.array(rospy.get_param("~robot_initial_pose", [0.0, 0.0]), dtype=float)
        self.robot_initial_vel = np.array(rospy.get_param("~robot_initial_velocity", [0.0, 0.0]), dtype=float)
        self.robot_initial_yaw_deg = float(rospy.get_param("~robot_initial_yaw_deg", 45.0))
        self.obstacle_yaw_deg = float(rospy.get_param("~obstacle_yaw_deg", 0.0))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 1.0))
        self.cooldown_s = float(rospy.get_param("~transition_cooldown_s", 2.0))
        self.pause_during_reset = bool(rospy.get_param("~pause_physics_during_reset", True))
        self.call_reset_simulation = bool(rospy.get_param("~call_reset_simulation", True))
        self.park_base = np.array(rospy.get_param("~park_base_xy", [500.0, 500.0]), dtype=float)
        self.park_spacing = float(rospy.get_param("~park_spacing", 10.0))

        sequence = rospy.get_param("~scenario_sequence", DEFAULT_SEQUENCE)
        self.sequence = [int(item) for item in sequence] if sequence else DEFAULT_SEQUENCE[:]
        if not self.sequence:
            self.sequence = DEFAULT_SEQUENCE[:]

        self.seq_index = 0
        self.current_scenario_id = self.sequence[self.seq_index]

        self._lock = threading.Lock()
        self._transitioning = False
        self._last_transition_time = rospy.Time(0)
        self._known_models = set()
        self._have_model_states = threading.Event()
        self._event_timers = []
        self._robot_state = RobotState()

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
            rospy.logwarn("[grouped_scenario_manager] pause/unpause services unavailable")

        try:
            rospy.wait_for_service("/gazebo/reset_simulation", timeout=2.0)
            self._reset_sim = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)
        except Exception:
            rospy.logwarn("[grouped_scenario_manager] reset_simulation service unavailable")

        self.robot_pub = rospy.Publisher("/scenario/input_robot", RobotState, queue_size=10)
        self.current_scenario_pub = rospy.Publisher("/current_scenario", String, queue_size=10)
        self.scenario_group_pub = rospy.Publisher("/scenario/group", Int32, queue_size=10)

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_cb, queue_size=1)
        rospy.Subscriber("/obstacle_avoidance/collision", Bool, self._collision_cb, queue_size=10)
        rospy.Subscriber("/scenario/reached_goal", Bool, self._reached_goal_cb, queue_size=10)
        rospy.Subscriber("/obstacle_avoidance/distance_to_goal", Float64, self._distance_to_goal_cb, queue_size=10)
        rospy.Subscriber("/grouped_scenario_manager/set_scenario", Int32, self._set_scenario_cb, queue_size=10)

        self._wait_first_model_states()
        self.apply_scenario(self.current_scenario_id, reason="startup")
        rospy.loginfo("[grouped_scenario_manager] started at scenario %d", self.current_scenario_id)

    def _wait_first_model_states(self):
        if not self._have_model_states.wait(timeout=5.0):
            rospy.logwarn("[grouped_scenario_manager] no /gazebo/model_states received yet")

    def _model_states_cb(self, msg):
        self._known_models = set(msg.name)
        self._have_model_states.set()

        for index, model_name in enumerate(msg.name):
            if model_name != ROBOT_NAME:
                continue
            self._robot_state.position = msg.pose[index].position
            self._robot_state.velocity = msg.twist[index].linear
            self._robot_state.orientation = msg.pose[index].orientation
            self._robot_state.radius = float(rospy.get_param("/apfm_obstacle_avoidance/robot_domain_radius", 1.4))
            self.robot_pub.publish(self._robot_state)
            break

    def _cooldown_ok(self):
        return (rospy.Time.now() - self._last_transition_time).to_sec() >= self.cooldown_s

    def _collision_cb(self, msg):
        if msg.data:
            self._request_advance("collision")

    def _reached_goal_cb(self, msg):
        if msg.data:
            self._request_advance("reached_goal")

    def _distance_to_goal_cb(self, msg):
        if msg.data <= self.goal_tolerance:
            self._request_advance("distance_to_goal")

    def _set_scenario_cb(self, msg):
        scenario_id = int(msg.data)
        if scenario_id not in SCENARIOS:
            rospy.logwarn("[grouped_scenario_manager] unknown scenario id=%s", scenario_id)
            return
        with self._lock:
            self.current_scenario_id = scenario_id
            if scenario_id in self.sequence:
                self.seq_index = self.sequence.index(scenario_id)
        self.apply_scenario(scenario_id, reason="manual_set")

    def _request_advance(self, reason):
        with self._lock:
            if self._transitioning or not self._cooldown_ok():
                return
            self._transitioning = True
        threading.Thread(target=self._advance_worker, args=(reason,), daemon=True).start()

    def _advance_worker(self, reason):
        try:
            with self._lock:
                self.seq_index = (self.seq_index + 1) % len(self.sequence)
                self.current_scenario_id = self.sequence[self.seq_index]
                next_id = self.current_scenario_id
            self.apply_scenario(next_id, reason="auto:%s" % reason)
        finally:
            with self._lock:
                self._transitioning = False
                self._last_transition_time = rospy.Time.now()

    def apply_scenario(self, scenario_id, reason=""):
        scenario_cfg = SCENARIOS.get(scenario_id)
        if scenario_cfg is None:
            rospy.logerr("[grouped_scenario_manager] scenario %s not defined", scenario_id)
            return

        self._cancel_pending_events()

        scenario_name = "group_%d_%02d_%s" % (
            scenario_cfg["group"],
            scenario_id,
            scenario_cfg["kind"],
        )
        rospy.set_param("/gazebo_scenario/scenario", scenario_name)
        rospy.set_param("/grouped_scenario_manager/current_group", int(scenario_cfg["group"]))
        rospy.set_param("/grouped_scenario_manager/current_scenario_id", int(scenario_id))

        self.current_scenario_pub.publish(String(data=scenario_name))
        self.scenario_group_pub.publish(Int32(data=int(scenario_cfg["group"])))

        rospy.loginfo(
            "[grouped_scenario_manager] applying %s (reason=%s): %s",
            scenario_name,
            reason,
            scenario_cfg["description"],
        )

        try:
            if self.pause_during_reset and self._pause is not None:
                self._pause()

            if self.call_reset_simulation and self._reset_sim is not None:
                self._reset_sim()
                rospy.sleep(0.2)

            self._set_if_exists(
                make_model_state(
                    ROBOT_NAME,
                    self.robot_initial_pose,
                    self.robot_initial_vel,
                    yaw_deg=self.robot_initial_yaw_deg,
                )
            )

            active_names = set(scenario_cfg["obstacles"].keys())
            park_positions = self._parking_positions()

            for index, obstacle_name in enumerate(OBSTACLE_NAMES):
                if obstacle_name in active_names:
                    obstacle_cfg = scenario_cfg["obstacles"][obstacle_name]
                    self._set_if_exists(
                        make_model_state(
                            obstacle_name,
                            obstacle_cfg["position"],
                            obstacle_cfg["velocity"],
                            yaw_deg=self.obstacle_yaw_deg,
                        )
                    )
                else:
                    park_xy = park_positions[index]
                    self._set_if_exists(
                        make_model_state(
                            obstacle_name,
                            park_xy,
                            [0.0, 0.0],
                            yaw_deg=self.obstacle_yaw_deg,
                        )
                    )
        except rospy.ServiceException as exc:
            rospy.logerr("[grouped_scenario_manager] failed to apply scenario: %s", exc)
        finally:
            if self.pause_during_reset and self._unpause is not None:
                try:
                    self._unpause()
                except Exception:
                    pass

        self._schedule_events_for_scenario(scenario_id, scenario_cfg)

    def _schedule_events_for_scenario(self, scenario_id, scenario_cfg):
        for event in scenario_cfg["events"]:
            timer = rospy.Timer(
                rospy.Duration(float(event["time"])),
                lambda _evt, sid=scenario_id, cfg=event: self._apply_event(sid, cfg),
                oneshot=True,
            )
            self._event_timers.append(timer)

    def _apply_event(self, scenario_id, event_cfg):
        if scenario_id != self.current_scenario_id:
            return

        rospy.loginfo(
            "[grouped_scenario_manager] event for scenario %d at %.2fs: %s",
            scenario_id,
            float(event_cfg["time"]),
            event_cfg.get("description", ""),
        )

        for obstacle_name, updates in event_cfg["updates"].items():
            current_position = updates.get("position")
            if current_position is None:
                current_position = self._lookup_model_position(obstacle_name)
            if current_position is None:
                rospy.logwarn("[grouped_scenario_manager] event skipped, model not found: %s", obstacle_name)
                continue

            velocity = updates.get("velocity", [0.0, 0.0])
            try:
                self._set_if_exists(
                    make_model_state(
                        obstacle_name,
                        current_position,
                        velocity,
                        yaw_deg=self.obstacle_yaw_deg,
                    )
                )
            except rospy.ServiceException as exc:
                rospy.logerr("[grouped_scenario_manager] event update failed for %s: %s", obstacle_name, exc)

    def _lookup_model_position(self, model_name):
        msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=1.0)
        for index, known_name in enumerate(msg.name):
            if known_name == model_name:
                return [msg.pose[index].position.x, msg.pose[index].position.y]
        return None

    def _cancel_pending_events(self):
        for timer in self._event_timers:
            timer.shutdown()
        self._event_timers = []

    def _set_if_exists(self, state):
        if self._known_models and state.model_name not in self._known_models:
            return
        response = self._set_state_srv(state)
        if not response.success:
            rospy.logwarn(
                "[grouped_scenario_manager] set_model_state failed for %s: %s",
                state.model_name,
                response.status_message,
            )

    def _parking_positions(self):
        positions = []
        base_x = float(self.park_base[0])
        base_y = float(self.park_base[1])
        for index in range(len(OBSTACLE_NAMES)):
            row = index // 5
            col = index % 5
            positions.append([base_x + col * self.park_spacing, base_y + row * self.park_spacing])
        return positions


if __name__ == "__main__":
    try:
        GroupedScenarioManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
