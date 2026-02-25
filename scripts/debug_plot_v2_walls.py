#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
debug_plot_v2_walls.py

Versão melhorada do seu debug_plot.py:
- Mostra claramente as PAREDES VIRTUAIS (obstáculos com name "wall_L_*" e "wall_R_*")
  como linhas/polilinhas + pontos.
- Mantém a ideia de "o que o robô está vendo" (usa /scenario/output_obstacles).
- Exibe: robô, direção (yaw), força total, goal, obstáculos (com raio), vetores de velocidade,
  círculos CR e dm (CustomInfo), e texto action/avoidance.
- Viewport centrado no robô (melhor para canal estreito).

Tópicos usados (iguais ao seu):
- /scenario/input_robot (RobotState)
- /scenario/output_obstacles (ObstacleArray)
- /apfm/total_force (geometry_msgs/Vector3)
- /scenario/goal (geometry_msgs/Vector3)
- /obstacle_avoidance/custom_info (CustomInfo)
Extra:
- /current_scenario (std_msgs/String) (opcional)

Como rodar:
  rosrun <seu_pkg> debug_plot_v2_walls.py

Dicas:
- Se você estiver rodando o gazebo_scenario_v2_dissertacao, as paredes já aparecem
  como obstáculos "wall_L_k"/"wall_R_k" no output_obstacles.
"""
import rospy
import numpy as np
import matplotlib.pyplot as plt

from dynamic_obstacle_avoidance.msg import RobotState, ObstacleArray, CustomInfo
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


EPS = 1e-9


def unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < EPS:
        return np.zeros_like(v, dtype=float)
    return v / n


class DebugPlotV2:
    def __init__(self):
        rospy.init_node('debug_plot_v2_walls')

        # Estado interno
        self.robot_position = None
        self.robot_orientation = None
        self.force_vector = None
        self.goal_position = None

        self.obstacles = []   # pos, vel, radius, name
        self.wall_L = []      # pontos da parede esquerda (virtual)
        self.wall_R = []      # pontos da parede direita (virtual)

        self.vector_to_obstacle = None
        self.relative_speed_vector = None
        self.collision_avoidance_radius = None
        self.center_to_center_safe_distance = None
        self.action_type = "N/A"
        self.avoidance_type = "N/A"
        self.current_scenario = ""

        # Parâmetros de visualização
        self.view_radius = float(rospy.get_param("~view_radius", 40.0))
        self.force_scale = float(rospy.get_param("~force_scale", 12.0))
        self.vel_scale = float(rospy.get_param("~vel_scale", 4.0))
        self.draw_obstacle_radii = bool(rospy.get_param("~draw_obstacle_radii", True))
        self.draw_wall_polyline = bool(rospy.get_param("~draw_wall_polyline", True))
        self.wall_point_size = float(rospy.get_param("~wall_point_size", 20.0))
        self.show_legend = bool(rospy.get_param("~show_legend", True))

        # Plot
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.grid(True, alpha=0.3)

        # Subscribers
        self.robot_sub = rospy.Subscriber('/scenario/input_robot', RobotState, self.robot_callback)
        self.obstacle_sub = rospy.Subscriber('/scenario/output_obstacles', ObstacleArray, self.obstacle_callback)
        self.force_sub = rospy.Subscriber('/apfm/total_force', Vector3, self.force_callback)
        self.goal_sub = rospy.Subscriber('/scenario/goal', Vector3, self.goal_callback)
        self.custom_info_sub = rospy.Subscriber('/obstacle_avoidance/custom_info', CustomInfo, self.custom_info_callback)
        self.scenario_sub = rospy.Subscriber('/current_scenario', String, self.scenario_callback)

    def scenario_callback(self, msg: String):
        self.current_scenario = msg.data

    def robot_callback(self, msg: RobotState):
        self.robot_position = np.array([msg.position.x, msg.position.y], dtype=float)
        self.robot_orientation = msg.orientation

    def obstacle_callback(self, msg: ObstacleArray):
        obs = []
        wall_L = []
        wall_R = []

        for ob in msg.obstacles:
            name = getattr(ob, "name", "")
            pos = np.array([ob.position.x, ob.position.y], dtype=float)
            vel = np.array([ob.velocity.x, ob.velocity.y], dtype=float)
            radius = float(ob.radius)

            if name.startswith("wall_L"):
                wall_L.append(pos)
            elif name.startswith("wall_R"):
                wall_R.append(pos)
            else:
                obs.append({"name": name, "pos": pos, "vel": vel, "radius": radius})

        self.obstacles = obs
        self.wall_L = wall_L
        self.wall_R = wall_R

    def force_callback(self, msg: Vector3):
        self.force_vector = np.array([msg.x, msg.y], dtype=float)

    def goal_callback(self, msg: Vector3):
        self.goal_position = np.array([msg.x, msg.y], dtype=float)

    def custom_info_callback(self, msg: CustomInfo):
        self.vector_to_obstacle = np.array(msg.vector_to_obstacle[:2], dtype=float) if len(msg.vector_to_obstacle) >= 2 else np.zeros(2)
        self.relative_speed_vector = np.array(msg.relative_speed_vector[:2], dtype=float) if len(msg.relative_speed_vector) >= 2 else np.zeros(2)
        self.collision_avoidance_radius = float(getattr(msg, "CR", 0.0))
        self.center_to_center_safe_distance = float(getattr(msg, "dm", 0.0))
        self.action_type = getattr(msg, "action_type", "N/A")
        self.avoidance_type = getattr(msg, "avoidance_type", "N/A")

    def _draw_arrow(self, origin_xy, vec_xy, color='k', label=None, scale=1.0, head=1.6):
        v = np.array(vec_xy, dtype=float)
        n = float(np.linalg.norm(v))
        if n < EPS:
            return
        v = v * scale
        self.ax.arrow(origin_xy[0], origin_xy[1], v[0], v[1],
                      head_width=head, head_length=head, length_includes_head=True,
                      fc=color, ec=color, linewidth=1.5, alpha=0.9, label=label)

    def _sort_points_along_axis(self, pts, axis_u):
        pts = [np.array(p, dtype=float) for p in pts]
        axis_u = unit(np.array(axis_u, dtype=float))
        if float(np.linalg.norm(axis_u)) < EPS:
            axis_u = np.array([1.0, 0.0], dtype=float)
        keys = [float(np.dot(p, axis_u)) for p in pts]
        return [p for _, p in sorted(zip(keys, pts), key=lambda t: t[0])]

    def _draw_wall(self, pts, color='0.25', label=None):
        if len(pts) == 0:
            return
        pts_arr = np.array(pts, dtype=float)
        self.ax.scatter(pts_arr[:, 0], pts_arr[:, 1], s=self.wall_point_size, c=color, marker='s', alpha=0.9, label=label)

        if self.draw_wall_polyline and len(pts) >= 2:
            if self.goal_position is not None and self.robot_position is not None:
                axis = self.goal_position - self.robot_position
            else:
                C = np.cov(pts_arr.T)
                w, V = np.linalg.eig(C)
                axis = V[:, int(np.argmax(w))]
            sorted_pts = self._sort_points_along_axis(pts, axis)
            xs = [p[0] for p in sorted_pts]
            ys = [p[1] for p in sorted_pts]
            self.ax.plot(xs, ys, linestyle='-', linewidth=2.0, alpha=0.8, color=color)

    def update_plot(self):
        self.ax.clear()
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')

        if self.robot_position is None:
            plt.pause(0.05)
            return

        x, y = float(self.robot_position[0]), float(self.robot_position[1])
        R = self.view_radius
        self.ax.set_xlim(x - R, x + R)
        self.ax.set_ylim(y - R, y + R)

        # Paredes virtuais
        self._draw_wall(self.wall_L, color='0.25', label="Wall Left (virtual)")
        self._draw_wall(self.wall_R, color='0.25', label="Wall Right (virtual)")

        # Robô
        self.ax.plot(x, y, 'bo', markersize=8, label="Robot")

        # Heading
        if self.robot_orientation is not None:
            q = [self.robot_orientation.x, self.robot_orientation.y, self.robot_orientation.z, self.robot_orientation.w]
            _, _, yaw = euler_from_quaternion(q)
            heading = np.array([np.cos(yaw), np.sin(yaw)], dtype=float)
            self._draw_arrow(self.robot_position, heading, color='r', label="Robot Heading", scale=6.0, head=1.2)

        # Força total (direção)
        if self.force_vector is not None:
            f = np.array(self.force_vector, dtype=float)
            n = float(np.linalg.norm(f))
            if n > EPS:
                f_unit = f / n
                self._draw_arrow(self.robot_position, f_unit, color='g', label="Total Force (dir)", scale=self.force_scale, head=1.6)

        # Goal
        if self.goal_position is not None:
            self.ax.plot(float(self.goal_position[0]), float(self.goal_position[1]), 'kx', markersize=10, label="Goal")

        # Debug vectors
        if self.vector_to_obstacle is not None and float(np.linalg.norm(self.vector_to_obstacle)) > 1e-6:
            self._draw_arrow(self.robot_position, self.vector_to_obstacle, color='m', label="Vector to obstacle", scale=1.0, head=1.2)

        if self.relative_speed_vector is not None and float(np.linalg.norm(self.relative_speed_vector)) > 1e-6:
            self._draw_arrow(self.robot_position, self.relative_speed_vector, color='c', label="Relative speed", scale=self.vel_scale, head=1.2)

        # Círculos CR e dm
        if self.collision_avoidance_radius is not None and self.collision_avoidance_radius > 1e-6:
            cr = plt.Circle((x, y), self.collision_avoidance_radius, color='y', fill=False, linestyle='--',
                            linewidth=1.6, alpha=0.8, label="CR")
            self.ax.add_patch(cr)

        if (self.center_to_center_safe_distance is not None and self.center_to_center_safe_distance > 1e-6
                and self.vector_to_obstacle is not None):
            ox = x + float(self.vector_to_obstacle[0])
            oy = y + float(self.vector_to_obstacle[1])
            dm = plt.Circle((ox, oy), self.center_to_center_safe_distance, color='r', fill=False, linestyle='--',
                            linewidth=1.6, alpha=0.8, label="dm")
            self.ax.add_patch(dm)

        # Obstáculos reais
        for ob in self.obstacles:
            p = ob["pos"]
            v = ob["vel"]
            r = float(ob["radius"])
            name = ob["name"]

            self.ax.plot(float(p[0]), float(p[1]), 'ro', markersize=6)

            if self.draw_obstacle_radii and r > 1e-3:
                circ = plt.Circle((float(p[0]), float(p[1])), r, color='r', fill=False, linestyle='-',
                                  linewidth=1.0, alpha=0.6)
                self.ax.add_patch(circ)

            nv = float(np.linalg.norm(v))
            if nv > 1e-6:
                v_unit = v / nv
                self._draw_arrow(p, v_unit, color='tab:red', label=None, scale=5.0, head=1.0)

            if name:
                self.ax.text(float(p[0]) + 1.0, float(p[1]) + 1.0, name, fontsize=8, alpha=0.85)

        # Texto informativo
        info_lines = [
            f"Scenario: {self.current_scenario}" if self.current_scenario else "Scenario: (n/a)",
            f"Action: {self.action_type}",
            f"Avoidance: {self.avoidance_type}",
        ]
        self.ax.text(0.02, 0.98, "\n".join(info_lines),
                     transform=self.ax.transAxes, va='top', ha='left',
                     fontsize=10, bbox=dict(facecolor='white', alpha=0.65, edgecolor='0.8'))

        if self.show_legend:
            handles, labels = self.ax.get_legend_handles_labels()
            uniq = {}
            for h, l in zip(handles, labels):
                if l and l not in uniq:
                    uniq[l] = h
            if uniq:
                self.ax.legend(uniq.values(), uniq.keys(), loc='lower right', fontsize=8, framealpha=0.85)

        plt.draw()
        plt.pause(0.001)

    def run(self):
        rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            self.update_plot()
            rate.sleep()


if __name__ == '__main__':
    DebugPlotV2().run()
