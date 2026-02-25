#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
collision_watchdog_v2.py

Correção principal:
- Ao detectar colisão geométrica (dist < R_os + R_ts), publica:
    /obstacle_avoidance/collision  (Bool=True)   <-- runner escuta isso para finalizar
    /metrics/collision_geom        (Bool=True)
  E publica mais de uma vez + pequeno sleep para garantir entrega antes de encerrar.

Também pode (opcionalmente) parar o robô e pausar a física do Gazebo.
"""

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Empty

from dynamic_obstacle_avoidance.msg import RobotState, ObstacleArray


class CollisionWatchdogV2:
    def __init__(self):
        # Fallbacks (se radius vier 0)
        self.robot_radius_fallback = float(rospy.get_param("~robot_radius_fallback", 1.4))
        self.obs_radius_fallback = float(rospy.get_param("~obs_radius_fallback", 1.0))

        # Ignorar paredes virtuais wall_*
        self.ignore_walls = bool(rospy.get_param("~ignore_walls", False))

        # Stop behavior
        self.stop_repeats = int(rospy.get_param("~stop_repeats", 10))
        self.stop_dt = float(rospy.get_param("~stop_dt", 0.05))

        # Delivery robustness
        self.publish_repeats = int(rospy.get_param("~publish_repeats", 5))
        self.publish_dt = float(rospy.get_param("~publish_dt", 0.02))

        # Gazebo pause (optional)
        self.pause_gazebo_physics = bool(rospy.get_param("~pause_gazebo_physics", True))
        self.pause_srv = None
        if self.pause_gazebo_physics:
            try:
                rospy.wait_for_service("/gazebo/pause_physics", timeout=2.0)
                self.pause_srv = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
            except Exception:
                self.pause_srv = None

        self.robot = None
        self.obstacles = None
        self.collided = False

        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Métrica: colisão geométrica
        self.pub_collision_geom = rospy.Publisher("/metrics/collision_geom", Bool, queue_size=10, latch=True)

        # Sinal para o runner finalizar e fechar tudo
        self.pub_collision_runner = rospy.Publisher("/obstacle_avoidance/collision", Bool, queue_size=10, latch=True)

        rospy.Subscriber("/scenario/output_robot", RobotState, self.cb_robot, queue_size=1)
        rospy.Subscriber("/scenario/output_obstacles", ObstacleArray, self.cb_obs, queue_size=1)

        rospy.Timer(rospy.Duration(0.05), self.on_timer)  # 20 Hz

    def cb_robot(self, msg: RobotState):
        self.robot = msg

    def cb_obs(self, msg: ObstacleArray):
        self.obstacles = msg.obstacles

    def _stop_robot_and_pause(self):
        # Zera cmd_vel
        z = Twist()
        for _ in range(max(1, self.stop_repeats)):
            self.pub_cmd.publish(z)
            rospy.sleep(self.stop_dt)

        # Pausa Gazebo
        if self.pause_srv is not None:
            try:
                self.pause_srv()
            except Exception:
                pass

    def _publish_collision_flags(self):
        # Publica múltiplas vezes p/ garantir que o runner receba (robusto contra shutdown imediato)
        for _ in range(max(1, self.publish_repeats)):
            self.pub_collision_geom.publish(Bool(data=True))
            self.pub_collision_runner.publish(Bool(data=True))
            rospy.sleep(self.publish_dt)

    def check_collision(self) -> bool:
        if self.robot is None or self.obstacles is None:
            return False

        px = float(self.robot.position.x)
        py = float(self.robot.position.y)

        R_os = float(getattr(self.robot, "radius", 0.0))
        if R_os <= 1e-6:
            R_os = self.robot_radius_fallback

        for ob in self.obstacles:
            name = str(getattr(ob, "name", ""))
            if self.ignore_walls and name.startswith("wall_"):
                continue

            ox = float(ob.position.x)
            oy = float(ob.position.y)

            R_ts = float(getattr(ob, "radius", 0.0))
            if R_ts <= 1e-6:
                R_ts = self.obs_radius_fallback

            d = math.hypot(ox - px, oy - py)
            if d < (R_os + R_ts):
                return True

        return False

    def on_timer(self, _evt):
        if self.collided:
            return

        self.collided = self.check_collision()

        if self.collided:
            rospy.logerr("[collision_watchdog_v2] COLISÃO detectada: dist < R_robo + R_obstaculo. Sinalizando runner e parando.")
            self._publish_collision_flags()
            self._stop_robot_and_pause()
            rospy.signal_shutdown("collision_geom")


if __name__ == "__main__":
    rospy.init_node("collision_watchdog")
    CollisionWatchdogV2()
    rospy.spin()