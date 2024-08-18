#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Wrench, Twist, Vector3
from nav_msgs.msg import Odometry
from dynamic_obstacle_avoidance.msg import RobotState
from tf.transformations import euler_from_quaternion
import numpy as np

class ForceCalculator:
    def __init__(self):
        rospy.init_node('force_calculator')

        # Constants
        self.MAX_WRENCH = rospy.get_param('~max_wrench', 0.3)
        self.MAX_TORQUE_WRENCH = rospy.get_param('~max_torque_wrench', 0.1)
        self.MIN_TORQUE_WRENCH = rospy.get_param('~min_torque_wrench', -0.1)
        self.MAX_LINEAR_SPEED = rospy.get_param('~max_linear_speed', 1.0)

        # Publishers
        self.twist_pub = rospy.Publisher('/apfm/cmd_vel', Twist, queue_size=10)
        self.wrench_pub = rospy.Publisher('/apfm/wrench', Wrench, queue_size=10)

        # Subscribers
        self.force_sub = rospy.Subscriber('/apfm/total_force', Vector3, self.force_callback)
        self.robot_sub = rospy.Subscriber('/scenario/output_robot',RobotState, self.robot_callback)
        
        # Initialize variables
        self.current_force = Vector3()
        self.time_step_counter = 0
        self.velocity_change_interval = 10  # Adjust as necessary
        self.obstacles = []  # Assume a list of obstacles with name and twist information

    def force_callback(self, msg):
        self.current_force = msg
        self.calculate_and_publish()

    def robot_callback(self, msg):
        self.robot_orientation = msg.orientation
        self.calculate_and_publish()

    def calculate_and_publish(self):
        force = self.current_force
        torque = Vector3() 

        
        orientation_list = [self.robot_orientation.x, self.robot_orientation.y, self.robot_orientation.z, self.robot_orientation.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        # Certifique-se de que ambos os ângulos estão no intervalo [-pi, pi]
        angle_force = np.arctan2(self.current_force.y, self.current_force.x)
        angle_force = (angle_force + np.pi) % (2 * np.pi) - np.pi

        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi

        # Calcule a diferença angular e ajuste para o intervalo [-pi, pi]
        angular_difference =   -(angle_force - yaw)
        angular_difference = (angular_difference + np.pi) % (2 * np.pi) - np.pi

        proportion_factor = 1.0
        torque_value = proportion_factor * angular_difference

        torque.x = 0.0
        torque.y = 0.0
        torque.z = torque_value

        # Log information
        rospy.loginfo("Calculou a força e o torque")

        wrench_msg = Wrench()
        twist_msg = Twist()

        wrench_msg.force.x = min(self.MAX_WRENCH, force.x)
        wrench_msg.torque.z = max(self.MIN_TORQUE_WRENCH, min(self.MAX_TORQUE_WRENCH, torque.z))
        twist_msg.linear.x = min(self.MAX_LINEAR_SPEED, force.x)
        twist_msg.angular.z = max(self.MIN_TORQUE_WRENCH, min(self.MAX_TORQUE_WRENCH, torque.z))

        self.twist_pub.publish(twist_msg)
        self.wrench_pub.publish(wrench_msg)


if __name__ == '__main__':
    try:
        ForceCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
