#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import numpy as np
from dynamic_obstacle_avoidance.msg import RobotState, ObstacleArray
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

class SimplePlot:
    def __init__(self):
        rospy.init_node('simple_plot')

        # Initialize variables
        self.robot_position = None
        self.robot_orientation = None
        self.force_vector = None
        self.goal_position = None
        self.obstacles = []

        # Initialize plot
        plt.ion()  # Turn on interactive mode
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-0, 50)
        self.ax.set_ylim(-0, 50)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        plt.show()

        # Subscribers
        self.robot_sub = rospy.Subscriber('/scenario/output_robot', RobotState, self.robot_callback)
        self.obstacle_sub = rospy.Subscriber('/scenario/output_obstacles', ObstacleArray, self.obstacle_callback)
        self.force_sub = rospy.Subscriber('/apfm/total_force', Vector3, self.force_callback)
        self.goal_sub = rospy.Subscriber('/scenario/goal', Vector3, self.goal_callback)

    def robot_callback(self, msg):
        self.robot_position = np.array([msg.position.x, msg.position.y])
        self.robot_orientation = msg.orientation

    def obstacle_callback(self, msg):
        self.obstacles = [
            {'position': np.array([ob.position.x, ob.position.y]), 'velocity': np.array([ob.velocity.x, ob.velocity.y])}
            for ob in msg.obstacles
        ]

    def force_callback(self, msg):
        self.force_vector = np.array([msg.x, msg.y])

    def goal_callback(self, msg):
        self.goal_position = np.array([msg.x, msg.y])

    def update_plot(self):
        self.ax.clear()

        # Plot robot position
        if self.robot_position is not None:
            self.ax.plot(self.robot_position[0], self.robot_position[1], 'bo', label="Robot")

            # Plot the force vector
            if self.force_vector is not None:
                normalized_force = self.force_vector / np.linalg.norm(self.force_vector)
                scaled_force = normalized_force * 10  # Scale the vector to a magnitude of 10 units
                self.ax.arrow(self.robot_position[0], self.robot_position[1],
                              scaled_force[0], scaled_force[1],
                              head_width=2, head_length=2, fc='g', ec='g', label="Force Vector")

                # Calculate the robot's direction vector from its orientation (yaw) and plot it
                orientation_q = [self.robot_orientation.x, self.robot_orientation.y, self.robot_orientation.z, self.robot_orientation.w]
                _, _, yaw = euler_from_quaternion(orientation_q)
                direction_vector = np.array([np.cos(yaw), np.sin(yaw)])

                # Scale the direction vector to half the size of the force vector
                scaled_direction = direction_vector * (np.linalg.norm(scaled_force) / 2)
                self.ax.arrow(self.robot_position[0], self.robot_position[1],
                              scaled_direction[0], scaled_direction[1],
                              head_width=1, head_length=1, fc='r', ec='r', label="Robot Direction")

        # Plot obstacles and their unitary velocity vectors
        for obs in self.obstacles:
            obs_pos = obs['position']
            obs_vel = obs['velocity']

            # Normalize the velocity vector to make it unitary if it's not zero
            norm = np.linalg.norm(obs_vel)
            if norm > 0:  # Only plot if the velocity is not zero
                unitary_vel = obs_vel / norm
                self.ax.arrow(obs_pos[0], obs_pos[1],
                              unitary_vel[0], unitary_vel[1],
                              head_width=1, head_length=1, fc='r', ec='r', label="Obstacle Velocity")

            self.ax.plot(obs_pos[0], obs_pos[1], 'ro', label="Obstacle")

        # Plot goal position
        if self.goal_position is not None:
            self.ax.plot(self.goal_position[0], self.goal_position[1], 'kx', markersize=10, label="Goal")

        self.ax.set_xlim(-0, 50)
        self.ax.set_ylim(-0, 50)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')

        plt.draw()
        plt.pause(0.001)

    def run(self):
        rate = rospy.Rate(1)  # Update the plot at 1 Hz
        while not rospy.is_shutdown():
            self.update_plot()
            rate.sleep()

if __name__ == '__main__':
    simple_plot = SimplePlot()
    simple_plot.run()
