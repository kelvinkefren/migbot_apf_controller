#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Point, Vector3, Quaternion
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from dynamic_obstacle_avoidance.msg import RobotState, ObstacleState, ObstacleArray

ROBOT_NAME = 'migbot'  # Nome do modelo do robô
OBSTACLE_NAMES = ['vegetation1_buoy','vegetation2_buoy','vegetation3_buoy','branche3_buoy']  # Nomes dos obstáculos
OBSTACLE_RADIUS = [1, 1, 1, 9, 7]  # Raios dos obstáculos
TOPIC_SUB = "/gazebo/model_states"  # Tópico do Gazebo para pegar Pose e Twist dos modelos

# Posição e velocidade iniciais do robô
INITIAL_ROBOT_POSE = np.array([0,30])  # Posição inicial do robô
INITIAL_ROBOT_VELOCITY = np.array([0,0])  # Velocidade inicial do robô

# Definição de cenários
SCENARIOS = {
    'scenario_0': {
        'vegetation1_buoy': {'position': [15, 12.6], 'velocity': [0, 1.5]},
        'vegetation2_buoy': {'position': [30, 20], 'velocity': [-0.5, 0,5]},
        'vegetation3_buoy': {'position': [60, 60], 'velocity': [0, 0]},
        'branche3_buoy': {'position': [60, 60], 'velocity': [0, 0]},
    },
}

class Obstacle:
    def __init__(self, name, radius):
        self.name = name
        self.radius = radius
        self.position = Point()
        self.velocity = Vector3()

    def update(self, pose, twist):
        self.position = pose.position
        self.velocity = twist.linear

class GazeboScenario:
    def __init__(self):
        rospy.init_node('migbot_controller')

        # Obter o nome do cenário a partir dos parâmetros ROS
        self.scenario_name = rospy.get_param('~scenario', 'scenario_0')

        # Inicializar o robô e os obstáculos
        self.migbot = RobotState()
        self.obstacles = [Obstacle(name, radius) for name, radius in zip(OBSTACLE_NAMES, OBSTACLE_RADIUS)]

        # Subscritor para o tópico de estados dos modelos do Gazebo
        self.model_state_sub = rospy.Subscriber(TOPIC_SUB, ModelStates, self.model_states_callback)

        # Publicadores para os tópicos de estados do robô e obstáculos
        self.robot_pub = rospy.Publisher('/scenario/input_robot', RobotState, queue_size=10)
        self.obstacle_pub = rospy.Publisher('/scenario/input_obstacles', ObstacleArray, queue_size=10)

        # Retrieve the robot domain radius parameter
        self.robot_domain_radius = rospy.get_param('/apfm_obstacle_avoidance/robot_domain_radius', default=0.5)
        
        # Configurar as posições e velocidades iniciais com base no cenário
        self.set_initial_positions()

    def set_initial_positions(self):
        # Configura a posição inicial e a velocidade do robô
        set_robot_position_and_velocity(INITIAL_ROBOT_POSE, INITIAL_ROBOT_VELOCITY)
        
        if self.scenario_name in SCENARIOS:
            scenario = SCENARIOS[self.scenario_name]
            for name, config in scenario.items():
                position = config['position']
                velocity = config['velocity']
                set_obstacle_position_and_velocity(name, position, velocity)

    def model_states_callback(self, data):
        for i, name in enumerate(data.name):
            if name == ROBOT_NAME:
                self.migbot.position = data.pose[i].position
                self.migbot.velocity = data.twist[i].linear
                self.migbot.orientation = data.pose[i].orientation
                self.migbot.radius = self.robot_domain_radius
                self.robot_pub.publish(self.migbot)
            elif name in OBSTACLE_NAMES:
                for ob in self.obstacles:
                    if ob.name == name:
                        ob.update(data.pose[i], data.twist[i])
                        break
        self.publish_obstacles()

    def publish_obstacles(self):
        obstacle_array = ObstacleArray()
        obstacle_array.obstacles = [
            ObstacleState(
                position=ob.position,
                velocity=ob.velocity,
                radius=radius
            ) for ob, radius in zip(self.obstacles, OBSTACLE_RADIUS)
        ]
        self.obstacle_pub.publish(obstacle_array)

def set_robot_position_and_velocity(position, velocity):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state = ModelState()
        state.model_name = ROBOT_NAME
        state.pose.position.x = position[0]
        state.pose.position.y = position[1]
        state.twist.linear.x = velocity[0]
        state.twist.linear.y = velocity[1]
        state.pose.orientation = Quaternion(0, 0, 0, 1)  # Sem rotação

        response = set_state(state)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False

def set_obstacle_position_and_velocity(name, position, velocity):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state = ModelState()
        state.model_name = name
        state.pose.position.x = position[0]
        state.pose.position.y = position[1]
        state.twist.linear.x = velocity[0]
        state.twist.linear.y = velocity[1]
        state.pose.orientation = Quaternion(0, 0, 0.3827, 0.9239)  # Sem rotação

        response = set_state(state)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False

if __name__ == '__main__':
    try:
        GazeboScenario()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
