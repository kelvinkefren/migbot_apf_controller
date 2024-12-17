#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Point, Vector3, Quaternion
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Bool
from dynamic_obstacle_avoidance.msg import RobotState, ObstacleState, ObstacleArray

ROBOT_NAME = 'migbot'  # Nome do modelo do robô
OBSTACLE_NAMES = ['vegetation1_buoy','vegetation3_buoy','branche3_buoy','vegetation3_buoy_clone','vegetation3_buoy_clone_clone','vegetation3_buoy_clone_clone_clone','branche3_buoy_clone_clone_clone_clone','trunk1_buoy','trunk1_buoy_clone','branche1_buoy']  # Nomes dos obstáculos
OBSTACLE_RADIUS = [1, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 3.15, 3.15, 4.4]  # Raios dos obstáculos
TOPIC_SUB = "/gazebo/model_states"  # Tópico do Gazebo para pegar Pose e Twist dos modelos

# Posição e velocidade iniciais do robô
INITIAL_ROBOT_POSE = np.array([0,0])  # Posição inicial do robô
INITIAL_ROBOT_VELOCITY = np.array([0,0])  # Velocidade inicial do robô

# Definição de cenários
SCENARIOS = {
    'scenario_A': {
        # Cenário A (Básico): Obst estático - SCENARIOS 1
        'vegetation3_buoy': {'position': [40,40], 'velocity':[0,0]},
    },
    'scenario_C': {
        # Cenário Congestionado - SCENARIOS 7 
        'vegetation3_buoy': {'position':[20,20], 'velocity':[0,0]},
        'vegetation3_buoy_clone': {'position':[30,35], 'velocity':[0,0]},
        'vegetation3_buoy_clone_clone': {'position':[50,50], 'velocity':[0,0]},
        'vegetation3_buoy_clone_clone_clone': {'position':[60,65], 'velocity':[0,0]},
        'branche3_buoy': {'position':[40,20], 'velocity':[0,0.4]},
        'branche3_buoy_clone': {'position':[25,50], 'velocity':[0.3,0]},
    },
    # Head-On Scenarios
    'scenario_headon_1': {'vegetation3_buoy': {'position': [10, -2.5], 'velocity': [0, 0.25]}},
    'scenario_headon_2': {'vegetation3_buoy': {'position': [15, -4], 'velocity': [-0.5, 0.4]}},
    'scenario_headon_3': {'vegetation3_buoy': {'position': [15, 4], 'velocity': [-0.5, -0.4]}},
    'scenario_headon_4': {'vegetation3_buoy': {'position': [20, 20], 'velocity': [-1.0, 0.0]}},
    'scenario_headon_5': {'vegetation3_buoy': {'position': [8, 2], 'velocity': [0.2, -0.2]}},
    'scenario_headon_6': {'vegetation3_buoy': {'position': [8, -2], 'velocity': [0.2, 0.2]}},
    # Crossing A Scenarios
    'scenario_crossingA_1': {'vegetation3_buoy': {'position': [14, -5.0], 'velocity': [-0.4, 0.5]}},
    'scenario_crossingA_2': {'vegetation3_buoy': {'position': [10.0, -10.0], 'velocity': [0.0, 1.0]}},
    'scenario_crossingA_3': {'vegetation3_buoy': {'position': [7.0, -4.0], 'velocity': [0.3, 0.4]}},
    'scenario_crossingA_4': {'vegetation3_buoy': {'position': [4.0, -6.5], 'velocity': [0.6, 0.7]}},
    'scenario_crossingA_5': {'vegetation3_buoy': {'position': [1.0, -3.5], 'velocity': [0.9, -0.37]}},
    'scenario_crossingA_6': {'vegetation3_buoy': {'position': [-2.0, -5.4], 'velocity': [1.22, -0.55]}},
    # Crossing B Scenarios
    'scenario_crossingB_1': {'vegetation3_buoy': {'position': [14, 5.0], 'velocity': [-0.4, -0.5]}},
    'scenario_crossingB_2': {'vegetation3_buoy': {'position': [10.0, 10.0], 'velocity': [0.0, -1.0]}},
    'scenario_crossingB_3': {'vegetation3_buoy': {'position': [7.0, 4.0], 'velocity': [0.3, -0.4]}},
    'scenario_crossingB_4': {'vegetation3_buoy': {'position': [4.0, 6.5], 'velocity': [0.6, -0.7]}},
    'scenario_crossingB_5': {'vegetation3_buoy': {'position': [1.0, 3.5], 'velocity': [0.9, 0.37]}},
    'scenario_crossingB_6': {'vegetation3_buoy': {'position': [-2.0, 5.4], 'velocity': [1.22, 0.55]}},

    #artigo:
    'scenario_from_table': {
        # Obstacles as per the table
        'vegetation3_buoy': {'position': [7.8, 2.2], 'velocity': [0, 0]},
        'vegetation3_buoy_clone': {'position': [6.8, 4.9], 'velocity': [0, 0]},
        'vegetation3_buoy_clone_clone': {'position': [7.0, 0], 'velocity': [-8, 8]},
        'vegetation3_buoy_clone_clone_clone': {'position': [7.5, 7], 'velocity': [-8, -8]},
        'vegetation3_buoy_clone_clone_clone_clone': {'position': [6, 8], 'velocity': [0, -3]},
        'vegetation3_buoy_trunk1_buoy': {'position': [4, 8], 'velocity': [2.8, -1.6]},
    },

    'scenario_teste_1': {'vegetation3_buoy': {'position': [39.85, 3.49], 'velocity': [-0.2, -0.05]}},
    'scenario_teste_2': {'vegetation3_buoy': {'position': [39.85, 3.49], 'velocity': [-0.2, 0.05]}},
    'scenario_teste_3': {'vegetation3_buoy': {'position': [39.85, 3.49], 'velocity': [0.2, -0.05]}},
    'scenario_teste_4': {'vegetation3_buoy': {'position': [39.85, 3.49], 'velocity': [0.2, 0.05]}},
    'scenario_teste_5': {'vegetation3_buoy': {'position': [39.85, -3.49], 'velocity': [-0.2, -0.05]}},
    'scenario_teste_6': {'vegetation3_buoy': {'position': [39.85, -3.49], 'velocity': [-0.2, 0.05]}},
    'scenario_teste_7': {'vegetation3_buoy': {'position': [39.85, -3.49], 'velocity': [0.2, -0.05]}},
    'scenario_teste_8': {'vegetation3_buoy': {'position': [39.85, -3.49], 'velocity': [0.2, 0.05]}},

    'scenario_teste_9': {'vegetation3_buoy': {'position': [28.28, -28.28], 'velocity': [0.5, 0.45]}},
    'scenario_teste_10': {'vegetation3_buoy': {'position': [28.28, -28.28], 'velocity': [0.5, 1.0]}},
    'scenario_teste_11': {'vegetation3_buoy': {'position': [28.28, 28.28], 'velocity': [0.6, -0.55]}},
    'scenario_teste_12': {'vegetation3_buoy': {'position': [28.28, 28.28], 'velocity': [0.6, -0.75]}},

    'scenario_teste_13': {'vegetation3_buoy': {'position': [-6.94, -39.39], 'velocity': [1.3, 1.1]}},
    'scenario_teste_14': {'vegetation3_buoy': {'position': [-6.94, -39.39], 'velocity': [1.6, 1.0]}},  #colisão
    'scenario_teste_15': {'vegetation3_buoy': {'position': [-13.68, 37.59], 'velocity': [1.3, -1.1]}},
    'scenario_teste_16': {'vegetation3_buoy': {'position': [-13.68, 37.59], 'velocity': [1.6, -1]}},

    'scenario_teste_17': {'vegetation3_buoy': {'position': [-38.64,-10.35], 'velocity': [2.5, 0.3]}},
    'scenario_teste_18': {'vegetation3_buoy': {'position': [-38.64,-10.35], 'velocity': [2.5, 0.7]}},
    'scenario_teste_19': {'vegetation3_buoy': {'position': [-38.64,10.35], 'velocity': [2.5, -0.3]}},
    'scenario_teste_20': {'vegetation3_buoy': {'position': [-38.64,10.35], 'velocity': [2.5, -0.7]}},
        
    'scenario_teste_21': {'vegetation3_buoy': {'position': [14.94, 1.31], 'velocity': [-0.2, -0.05]}},
    'scenario_teste_22': {'vegetation3_buoy': {'position': [14.94, 1.31], 'velocity': [-0.2, 0.05]}},
    'scenario_teste_23': {'vegetation3_buoy': {'position': [14.94, 1.31], 'velocity': [0.2, -0.05]}},
    'scenario_teste_24': {'vegetation3_buoy': {'position': [14.94, 1.31], 'velocity': [0.2, 0.05]}},
    'scenario_teste_25': {'vegetation3_buoy': {'position': [14.94, -1.31], 'velocity': [-0.2, -0.05]}},
    'scenario_teste_26': {'vegetation3_buoy': {'position': [14.94, -1.31], 'velocity': [-0.2, 0.05]}},
    'scenario_teste_27': {'vegetation3_buoy': {'position': [14.94, -1.31], 'velocity': [0.2, -0.05]}},
    'scenario_teste_28': {'vegetation3_buoy': {'position': [14.94, -1.31], 'velocity': [0.2, 0.05]}},

    'scenario_teste_29': {'vegetation3_buoy': {'position': [10.61, -10.61], 'velocity': [0.5, 0.45]}},
    'scenario_teste_30': {'vegetation3_buoy': {'position': [10.61, -10.61], 'velocity': [0.5, 1.0]}},
    'scenario_teste_31': {'vegetation3_buoy': {'position': [10.61, 10.61], 'velocity': [0.6, -0.55]}},
    'scenario_teste_32': {'vegetation3_buoy': {'position': [10.61, 10.61], 'velocity': [0.6, -0.75]}},

    'scenario_teste_33': {'vegetation3_buoy': {'position': [-2.60, -14.80], 'velocity': [1.3, 1.1]}},
    'scenario_teste_34': {'vegetation3_buoy': {'position': [-2.60, -14.80], 'velocity': [1.6, 1.0]}},  # colisão
    'scenario_teste_35': {'vegetation3_buoy': {'position': [-5.13, 14.10], 'velocity': [1.3, -1.1]}},
    'scenario_teste_36': {'vegetation3_buoy': {'position': [-5.13, 14.10], 'velocity': [1.6, -1]}},

    'scenario_teste_37': {'vegetation3_buoy': {'position': [-14.49, -3.88], 'velocity': [2.5, 0.3]}},
    'scenario_teste_38': {'vegetation3_buoy': {'position': [-14.49, -3.88], 'velocity': [2.5, 0.7]}},
    'scenario_teste_39': {'vegetation3_buoy': {'position': [-14.49, 3.88], 'velocity': [2.5, -0.3]}},
    'scenario_teste_40': {'vegetation3_buoy': {'position': [-14.49, 3.88], 'velocity': [2.5, -0.7]}},


    
}

class Obstacle:
    def __init__(self, name, radius):
        self.name = name
        self.radius = radius
        self.position = Point()
        self.velocity = Vector3()
        self.found = False

    def update(self, pose, twist):
        self.position = pose.position
        self.velocity = twist.linear
        self.found = True

class GazeboScenario:
    def __init__(self):
        rospy.init_node('gazebo_scenario')
        self.rotation = -60
        # Obter o nome do cenário a partir dos parâmetros ROS
        self.scenario_name = rospy.get_param('~scenario', 'scenario_teste_3')

        # Configurar um timer para verificar alterações no parâmetro
        rospy.Timer(rospy.Duration(1.0), self.check_for_parameter_update)

        # Inicializar o robô e os obstáculos
        self.migbot = RobotState()
        self.obstacles = [Obstacle(name, radius) for name, radius in zip(OBSTACLE_NAMES, OBSTACLE_RADIUS)]

        # Subscriber to the /obstacles topic
        rospy.Subscriber('/obstacles', ObstacleArray, self.obstacles_callback)

        # Publicadores para os tópicos de estados do robô e obstáculos
        self.robot_pub = rospy.Publisher('/scenario/input_robot', RobotState, queue_size=10)
        self.obstacle_pub = rospy.Publisher('/scenario/input_obstacles', ObstacleArray, queue_size=10)

        # Timer to rotate velocities after 5 seconds
        # rospy.Timer(rospy.Duration(5), self.rotate_obstacle_velocities, oneshot=True)
        
        # Retrieve the robot domain radius parameter
        self.robot_domain_radius = rospy.get_param('/apfm_obstacle_avoidance/robot_domain_radius', 1.4)
        rospy.loginfo(f"robot_domain_radius : {self.robot_domain_radius}")
        # Configurar as posições e velocidades iniciais com base no cenário
        self.set_initial_positions()
        
        # Subscritor para o tópico de estados dos modelos do Gazebo
        self.model_state_sub = rospy.Subscriber(TOPIC_SUB, ModelStates, self.model_states_callback)
        # Timer to rotate velocities after 5 seconds
        # rospy.Timer(rospy.Duration(5), self.rotate_obstacle_velocities, oneshot=True)
        
    def check_for_parameter_update(self, event):
        new_scenario_name = rospy.get_param('~scenario', self.scenario_name)
        if new_scenario_name != self.scenario_name:
            rospy.loginfo(f"Alterando cenário de {self.scenario_name} para {new_scenario_name}")
            self.scenario_name = new_scenario_name
            self.set_initial_positions()

    def obstacles_callback(self, data):
        # Implement your logic here
        pass

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

        for ob in self.obstacles:
            ob.found = False

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
                radius=ob.radius,      # Usando o raio armazenado no objeto Obstacle
                name=ob.name           # Adicionando o nome do obstáculo
            )
            for ob in self.obstacles if ob.found  # Inclui apenas se 'found' é True
        ]
        self.obstacle_pub.publish(obstacle_array)
        
    def rotate_obstacle_velocities(self, event):
        # Rotation matrix for 60 degrees clockwise
        angle_rad = np.radians(self.rotation)
        rotation_matrix = np.array([
            [np.cos(angle_rad), np.sin(angle_rad)],
            [-np.sin(angle_rad), np.cos(angle_rad)]
        ])

        for ob in self.obstacles:
            velocity_vector = np.array([ob.velocity.x, ob.velocity.y])
            rotated_velocity = rotation_matrix.dot(velocity_vector)
            ob.velocity.x = rotated_velocity[0]
            ob.velocity.y = rotated_velocity[1]
            set_obstacle_position_and_velocity(ob.name, [ob.position.x, ob.position.y], rotated_velocity)
        rospy.loginfo("Obstacle velocities rotated by 60 degrees clockwise")

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
