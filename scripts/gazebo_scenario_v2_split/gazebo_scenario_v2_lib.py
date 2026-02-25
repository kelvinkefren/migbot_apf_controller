#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_scenario_v2_lib.py (NO-CLONES)

Versão do runner v2 onde **NÃO** existem modelos *_clone* no seu mundo Gazebo.

Publica:
  - /scenario/input_robot (RobotState)
  - /scenario/input_obstacles (ObstacleArray)

Lê ground-truth via /gazebo/model_states e seta poses via /gazebo/set_model_state.

Obs:
- Se no seu mundo os nomes forem diferentes, você pode sobrescrever via parâmetro:
    ~obstacle_names := ["name1","name2",...]
    ~obstacle_radius := [r1,r2,...]  (mesmo tamanho)
"""
import math
import numpy as np
import rospy

from geometry_msgs.msg import Point, Vector3, Quaternion
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler

from dynamic_obstacle_avoidance.msg import RobotState, ObstacleState, ObstacleArray

ROBOT_NAME = 'migbot'

DEFAULT_OBSTACLE_NAMES = [
    'vegetation1_buoy',
    'vegetation3_buoy',
    'branche3_buoy',
    'trunk1_buoy',
    'branche1_buoy',
]
DEFAULT_OBSTACLE_RADIUS = [
    1.0,
    0.5,
    0.5,
    3.15,
    4.4,
]

TOPIC_MODEL_STATES = "/gazebo/model_states"
SRV_SET_STATE = "/gazebo/set_model_state"

DEFAULT_START = np.array([0.0, 0.0], dtype=float)
DEFAULT_GOAL  = np.array([70.0, 70.0], dtype=float)

EPS = 1e-9

def unit(v):
    n = float(np.linalg.norm(v))
    if n < EPS:
        return np.array([0.0, 0.0], dtype=float)
    return v / n

def rot90_ccw(v):
    return np.array([-v[1], v[0]], dtype=float)

def closest_point_on_segment(P, A, B):
    P = np.array(P, dtype=float)
    A = np.array(A, dtype=float)
    B = np.array(B, dtype=float)
    AB = B - A
    denom = float(np.dot(AB, AB))
    if denom < EPS:
        return A
    t = float(np.dot(P - A, AB) / denom)
    t = max(0.0, min(1.0, t))
    return A + t * AB

def polyline_to_segments(poly):
    segs = []
    for i in range(len(poly)-1):
        ax, ay = poly[i]
        bx, by = poly[i+1]
        segs.append((ax, ay, bx, by))
    return segs

def build_straight_channel(start_xy, goal_xy, width_m, margin_m=10.0, pieces=1):
    s = np.array(start_xy, dtype=float)
    g = np.array(goal_xy, dtype=float)
    u = unit(g - s)
    if float(np.linalg.norm(u)) < EPS:
        u = np.array([1.0, 0.0], dtype=float)
    n = rot90_ccw(u)
    A0 = s - margin_m*u
    A1 = g + margin_m*u
    pts = [A0 + (A1-A0)*(k/pieces) for k in range(pieces+1)]
    half = 0.5*width_m
    left_poly  = [(float(p[0] + half*n[0]), float(p[1] + half*n[1])) for p in pts]
    right_poly = [(float(p[0] - half*n[0]), float(p[1] - half*n[1])) for p in pts]
    return polyline_to_segments(left_poly), polyline_to_segments(right_poly)

def build_s_curve_channel(start_xy, goal_xy, width_m, margin_m=10.0):
    s = np.array(start_xy, dtype=float)
    g = np.array(goal_xy, dtype=float)
    u = unit(g - s)
    n = rot90_ccw(u)
    L = float(np.linalg.norm(g - s))
    if L < 1e-3:
        L = 100.0
    p0 = s - margin_m*u
    p1 = s + 0.33*L*u + 0.25*width_m*n
    p2 = s + 0.66*L*u - 0.25*width_m*n
    p3 = g + margin_m*u
    center_poly = [(float(p[0]), float(p[1])) for p in [p0,p1,p2,p3]]
    left_poly  = []
    right_poly = []
    for i in range(len(center_poly)-1):
        A = np.array(center_poly[i], dtype=float)
        B = np.array(center_poly[i+1], dtype=float)
        ui = unit(B - A)
        ni = rot90_ccw(ui)
        half = 0.5*width_m
        if i == 0:
            left_poly.append((float(A[0] + half*ni[0]), float(A[1] + half*ni[1])))
            right_poly.append((float(A[0] - half*ni[0]), float(A[1] - half*ni[1])))
        left_poly.append((float(B[0] + half*ni[0]), float(B[1] + half*ni[1])))
        right_poly.append((float(B[0] - half*ni[0]), float(B[1] - half*ni[1])))
    return polyline_to_segments(left_poly), polyline_to_segments(right_poly)

def set_model_pose_twist(model_name, pos_xy, vel_xy, yaw_deg=0.0, vel_scale=2.0):
    rospy.wait_for_service(SRV_SET_STATE)
    set_state = rospy.ServiceProxy(SRV_SET_STATE, SetModelState)
    state = ModelState()
    state.model_name = model_name
    state.pose.position.x = float(pos_xy[0])
    state.pose.position.y = float(pos_xy[1])
    state.pose.position.z = 0.0
    yaw = math.radians(float(yaw_deg))
    q = quaternion_from_euler(0.0, 0.0, yaw)
    state.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    state.twist.linear.x = float(vel_scale*vel_xy[0])
    state.twist.linear.y = float(vel_scale*vel_xy[1])
    state.twist.linear.z = 0.0
    state.twist.angular.x = 0.0
    state.twist.angular.y = 0.0
    state.twist.angular.z = 0.0
    resp = set_state(state)
    return bool(resp.success)

def compute_dm_ref(robot_domain_radius, safe_distance, wall_radius=0.0):
    return float(robot_domain_radius + safe_distance + wall_radius)

class GazeboScenarioV2Runner:
    def __init__(self, scenario_name, cfg_builder):
        rospy.init_node('gazebo_scenario')
        self.scenario_name = str(scenario_name)
        self.cfg_builder = cfg_builder

        self.vel_scale = float(rospy.get_param('~vel_scale', 2.0))
        self.wall_radius = float(rospy.get_param('~wall_radius', 0.0))
        self.wall_influence = float(rospy.get_param('~wall_influence', 60.0))
        self.enable_virtual_walls = bool(rospy.get_param('~enable_virtual_walls', True))

        self.start = np.array(rospy.get_param('~start_xy', DEFAULT_START.tolist()), dtype=float)
        self.goal  = np.array(rospy.get_param('~goal_xy',  DEFAULT_GOAL.tolist()), dtype=float)

        robot_domain_radius = float(rospy.get_param('/apfm_obstacle_avoidance/robot_domain_radius', 1.4))
        safe_distance = float(rospy.get_param('/apfm_obstacle_avoidance/safe_distance', 10.0))
        self.dm_ref = compute_dm_ref(robot_domain_radius, safe_distance, self.wall_radius)

        names = rospy.get_param('~obstacle_names', DEFAULT_OBSTACLE_NAMES)
        radii = rospy.get_param('~obstacle_radius', DEFAULT_OBSTACLE_RADIUS)
        if len(names) != len(radii):
            rospy.logwarn('[gazebo_scenario_v2_noclones] ~obstacle_names e ~obstacle_radius com tamanhos diferentes. Usando defaults.')
            names = DEFAULT_OBSTACLE_NAMES
            radii = DEFAULT_OBSTACLE_RADIUS

        self.OBSTACLE_NAMES = list(names)
        self.OBSTACLE_RADIUS = [float(r) for r in radii]

        self.cfg = self.cfg_builder(self.start, self.goal, self.dm_ref)

        self.change_velocity = False
        rospy.Subscriber('/change_velocity', Bool, self._change_velocity_cb)

        self.robot_pub = rospy.Publisher('/scenario/input_robot', RobotState, queue_size=10)
        self.obstacle_pub = rospy.Publisher('/scenario/input_obstacles', ObstacleArray, queue_size=10)

        self.model_sub = rospy.Subscriber(TOPIC_MODEL_STATES, ModelStates, self._model_states_cb)

        self.robot_state = RobotState()
        self.obstacles = {name: dict(radius=r, found=False, pos=Point(), vel=Vector3())
                          for name, r in zip(self.OBSTACLE_NAMES, self.OBSTACLE_RADIUS)}

        self._apply_scenario_fixed()
        rospy.loginfo(f'[gazebo_scenario_v2_noclones] Rodando cenário: {self.scenario_name}')

    def _change_velocity_cb(self, msg: Bool):
        self.change_velocity = bool(msg.data)

    def _apply_scenario_fixed(self):
        cfg = self.cfg
        rp = np.array(cfg['robot']['pos'], dtype=float)
        rv = np.array(cfg['robot']['vel'], dtype=float)
        ry = float(cfg['robot'].get('yaw_deg', 45.0))
        if not set_model_pose_twist(ROBOT_NAME, rp, rv, ry, vel_scale=self.vel_scale):
            rospy.logwarn('[gazebo_scenario_v2_noclones] Falha ao setar robô via /gazebo/set_model_state')

        for obs_name, obs_cfg in cfg.get('obstacles', {}).items():
            if obs_name not in self.obstacles:
                rospy.logwarn(f"[gazebo_scenario_v2_noclones] Obstáculo '{obs_name}' não está na lista ~obstacle_names. Ignorando.")
                continue
            op = np.array(obs_cfg['pos'], dtype=float)
            ov = np.array(obs_cfg['vel'], dtype=float)
            if not set_model_pose_twist(obs_name, op, ov, 0.0, vel_scale=self.vel_scale):
                rospy.logwarn(f'[gazebo_scenario_v2_noclones] Falha ao setar obstáculo {obs_name}')

        used = set(cfg.get('obstacles', {}).keys())
        far = np.array([200.0, 200.0], dtype=float)
        for obs_name in self.obstacles.keys():
            if obs_name in used:
                continue
            set_model_pose_twist(obs_name, far, np.array([0.0,0.0]), 0.0, vel_scale=self.vel_scale)

        self._event_timers = []
        for ev in cfg.get('events', []):
            t = float(ev.get('t', 0.0))
            self._event_timers.append(
                rospy.Timer(rospy.Duration(t),
                            lambda e, ev=ev: self._run_event(ev),
                            oneshot=True)
            )

    def _run_event(self, ev: dict):
        typ = ev.get('type', '')
        target = ev.get('target', '')
        if not target or target not in self.obstacles:
            rospy.logwarn(f'[gazebo_scenario_v2_noclones] Evento com target inválido: {target}')
            return

        pos = np.array([self.obstacles[target]['pos'].x, self.obstacles[target]['pos'].y], dtype=float)
        vel = np.array([self.obstacles[target]['vel'].x, self.obstacles[target]['vel'].y], dtype=float) / max(self.vel_scale, 1e-6)

        if typ == 'scale_velocity':
            scale = float(ev.get('scale', 1.0))
            set_model_pose_twist(target, pos, vel*scale, 0.0, vel_scale=self.vel_scale)
        elif typ == 'set_velocity':
            new_vel = np.array(ev.get('vel', [0.0,0.0]), dtype=float)
            set_model_pose_twist(target, pos, new_vel, 0.0, vel_scale=self.vel_scale)
        elif typ == 'rotate_velocity':
            ang_deg = float(ev.get('angle_deg', 30.0))
            ang = math.radians(ang_deg)
            R = np.array([[math.cos(ang), -math.sin(ang)],
                          [math.sin(ang),  math.cos(ang)]], dtype=float)
            set_model_pose_twist(target, pos, R.dot(vel), 0.0, vel_scale=self.vel_scale)
        else:
            rospy.logwarn(f'[gazebo_scenario_v2_noclones] Evento desconhecido: {ev}')

    def _model_states_cb(self, msg: ModelStates):
        for name in self.obstacles.keys():
            self.obstacles[name]['found'] = False

        for i, name in enumerate(msg.name):
            if name == ROBOT_NAME:
                self.robot_state.position = msg.pose[i].position
                self.robot_state.velocity = msg.twist[i].linear
                self.robot_state.orientation = msg.pose[i].orientation
                self.robot_state.radius = float(rospy.get_param('/apfm_obstacle_avoidance/robot_domain_radius', 1.4))
                self.robot_pub.publish(self.robot_state)
            elif name in self.obstacles:
                self.obstacles[name]['pos'] = msg.pose[i].position
                self.obstacles[name]['vel'] = msg.twist[i].linear
                self.obstacles[name]['found'] = True

        self._publish_obstacles_with_virtual_walls()

    def _publish_obstacles_with_virtual_walls(self):
        out = ObstacleArray()
        for name, st in self.obstacles.items():
            if not st['found']:
                continue
            out.obstacles.append(
                ObstacleState(
                    position=st['pos'],
                    velocity=st['vel'],
                    radius=float(st['radius']),
                    name=str(name)
                )
            )

        walls_cfg = self.cfg.get('walls', {})
        if self.enable_virtual_walls and bool(walls_cfg.get('enabled', False)):
            P = np.array([self.robot_state.position.x, self.robot_state.position.y], dtype=float)

            width = float(walls_cfg.get('width', 40.0))
            pieces = int(walls_cfg.get('pieces', 2))
            kind = str(walls_cfg.get('kind', 'straight'))

            if kind == 's_curve':
                left_segs, right_segs = build_s_curve_channel(self.start, self.goal, width)
            else:
                left_segs, right_segs = build_straight_channel(self.start, self.goal, width, pieces=pieces)

            def add_wall(segs, prefix):
                for k, (ax,ay,bx,by) in enumerate(segs):
                    Q = closest_point_on_segment(P, (ax,ay), (bx,by))
                    d = float(np.linalg.norm(P - Q))
                    if d <= self.wall_influence:
                        out.obstacles.append(
                            ObstacleState(
                                position=Point(x=float(Q[0]), y=float(Q[1]), z=0.0),
                                velocity=Vector3(x=0.0, y=0.0, z=0.0),
                                radius=float(self.wall_radius),
                                name=f'{prefix}_{k}'
                            )
                        )
            add_wall(left_segs,  'wall_L')
            add_wall(right_segs, 'wall_R')

        self.obstacle_pub.publish(out)

    def spin(self):
        rospy.spin()
