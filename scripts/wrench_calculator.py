#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Wrench, Twist, Vector3
from nav_msgs.msg import Odometry
from dynamic_obstacle_avoidance.msg import RobotState, CustomInfo
from tf.transformations import euler_from_quaternion
import numpy as np

class ForceCalculator:
    def __init__(self):
        rospy.init_node('force_calculator')
        
        # Initialize goal_position with a default value2
        self.goal_position = Vector3()
        # Constants
        self.MAX_WRENCH = rospy.get_param('~max_wrench', 0.2)
        self.MAX_TORQUE_WRENCH = rospy.get_param('~max_torque_wrench', 0.1)
        self.MIN_TORQUE_WRENCH = rospy.get_param('~min_torque_wrench', -0.1)
        self.MAX_LINEAR_SPEED = rospy.get_param('~max_linear_speed', 1.0)
        self.TOLERANCE = rospy.get_param('~tolerance', 5.0)

        # Publishers
        self.twist_pub = rospy.Publisher('/apfm/cmd_vel', Twist, queue_size=10)
        self.wrench_pub = rospy.Publisher('/apfm/wrench', Wrench, queue_size=10)

        rospy.Timer(rospy.Duration(0.1), self.publisher_wrench)

        # Subscribers
        self.force_sub = rospy.Subscriber('/apfm/total_force', Vector3, self.force_callback)
        self.robot_sub = rospy.Subscriber('/scenario/output_robot', RobotState, self.robot_callback)
        self.goal_sub = rospy.Subscriber('/scenario/goal', Vector3, self.goal_callback)
        self.avoidance_info = rospy.Subscriber('/obstacle_avoidance/custom_info',CustomInfo,self.info_callback)

        # Initialize variables
        self.current_force = Vector3()
        self.distance_to_goal = 0.0

    def info_callback(self,msg):
        self.distance_to_goal = msg.distance_to_goal
        
    def force_callback(self, msg):        
        self.current_force = msg
        # rospy.loginfo(f"Received force: {msg}, type: {type(msg)}")

    def robot_callback(self, msg):
        self.robot_orientation = msg.orientation
        self.robot_position = msg.position

        # Convert quaternion to roll, pitch, yaw
        orientation_list = [self.robot_orientation.x, self.robot_orientation.y, self.robot_orientation.z, self.robot_orientation.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        # Convert yaw (heading) to degrees
        yaw_degrees = math.degrees(yaw)

        # rospy.loginfo(f"Received robot state - Position: {self.robot_position}, Orientation (yaw): {yaw_degrees} degrees")

    def goal_callback(self, msg):
        self.goal_position = msg
        # rospy.loginfo(f"Received goal position: {self.goal_position}")

    def publisher_wrench(self, event):
        force = self.current_force
        torque = Vector3() 

        orientation_list = [self.robot_orientation.x, self.robot_orientation.y, self.robot_orientation.z, self.robot_orientation.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        yaw_degrees = math.degrees(yaw)
        # rospy.loginfo(f"Robot heading (yaw): {yaw_degrees} degrees")

        # Certifique-se de que ambos os ângulos estão no intervalo [-pi, pi]
        angle_force = np.arctan2(force.y, force.x)
        angle_force = (angle_force + np.pi) % (2 * np.pi) - np.pi
        angle_force_degrees = math.degrees(angle_force)

        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        yaw_degrees = math.degrees(yaw)

        # Calcule a diferença angular e ajuste para o intervalo [-pi, pi]
        angular_difference = -(angle_force - yaw)
        angular_difference = (angular_difference + np.pi) % (2 * np.pi) - np.pi
        angular_difference_degree = math.degrees(angular_difference)

        # rospy.loginfo(f"Force direction: {angle_force_degrees} degrees, Angular difference: {angular_difference_degree} degrees")

        proportion_factor = 1.0
        torque_value = proportion_factor * angular_difference

        torque.x = 0.0
        torque.y = 0.0
        torque.z = torque_value

        wrench_msg = Wrench()
        twist_msg = Twist()
        

        if self.distance_to_goal <= self.TOLERANCE:
            force.x = 0.0
            torque.z = 0.0
        else:
            None
        n = 4
        # Calcular o efeito angular (1 próximo de 0 ou 180 graus, 0 próximo de 90 graus)
        angular_effect = abs(math.cos(math.radians(angular_difference_degree)))**n

        rospy.loginfo(f"angular_effect : {angular_effect}")
        # # rospy.loginfo("**************************************************************************")
        # # rospy.loginfo(f"FORCE : {force}")
        # wrench_msg.force.x = min(self.MAX_WRENCH, abs(force.x)) * (1 if force.x >= 0 else 0)
        wrench_msg.force.x = angular_effect * min(self.MAX_WRENCH, abs(force.x))
        wrench_msg.torque.z = max(self.MIN_TORQUE_WRENCH, min(self.MAX_TORQUE_WRENCH, torque.z))
        twist_msg.linear.x = min(self.MAX_LINEAR_SPEED, force.x)
        twist_msg.angular.z = max(self.MIN_TORQUE_WRENCH, min(self.MAX_TORQUE_WRENCH, torque.z))
        
        # rospy.loginfo(f"Distance to goal: {distance_to_goal}")


        self.twist_pub.publish(twist_msg)
        self.wrench_pub.publish(wrench_msg)

        # rospy.loginfo(f"Published wrench: {wrench_msg}")
        # rospy.loginfo(f"Published twist: {twist_msg}")

if __name__ == '__main__':
    try:
        ForceCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
