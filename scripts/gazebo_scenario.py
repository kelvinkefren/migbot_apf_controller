#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Point, Vector3, Quaternion
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Bool
from dynamic_obstacle_avoidance.msg import RobotState, ObstacleState, ObstacleArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion

ROBOT_NAME = 'migbot'  # Nome do modelo do robô
OBSTACLE_NAMES = [
    'vegetation1_buoy','vegetation3_buoy','branche3_buoy',
    'vegetation3_buoy_clone','vegetation3_buoy_clone_clone','vegetation3_buoy_clone_clone_clone',
    'branche3_buoy_clone_clone_clone_clone','trunk1_buoy','trunk1_buoy_clone','branche1_buoy'
]  # Nomes dos obstáculos
OBSTACLE_RADIUS = [1, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 3.15, 3.15, 4.4]  # Raios dos obstáculos
TOPIC_SUB = "/gazebo/model_states"  # Tópico do Gazebo para pegar Pose e Twist dos modelos

# Posição e velocidade iniciais do robô
INITIAL_ROBOT_POSE = np.array([0,0])  # Posição inicial do robô
INITIAL_ROBOT_VELOCITY = np.array([0,0])  # Velocidade inicial do robô
INITIAL_ROBOT_ORIENTATION = 45 #GRAUS rotacionado para a esquerda (anti-horário)

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

    #dissertacao 
    'sceinario_dissertacao_1': {'vegetation3_buoy': {'position': [44, 44], 'velocity': [0, 0]}}, #estático
    'sceinario_dissertacao_2': {'vegetation3_buoy': {'position': [60, 60], 'velocity': [-0.85, -0.85]}}, #frontal
    'sceinario_dissertacao_3': {'vegetation3_buoy': {'position': [50, 0], 'velocity': [0, 0.85]}}, #Cross A
    'sceinario_dissertacao_4': {'vegetation3_buoy': {'position': [50, 0], 'velocity': [-0.2, 0.95]}}, #crossing A, efetuar manobra
    'sceinario_dissertacao_5': {'vegetation3_buoy': {'position': [0, 50], 'velocity': [0.85, 0]}},
    'sceinario_dissertacao_6': {'vegetation3_buoy': {'position': [0, 50], 'velocity': [0.7, -0.2]}},
    'sceinario_dissertacao_7': {'vegetation3_buoy': {'position': [-10, -10], 'velocity': [1.5, 1.5]}},

    'sceinario_dissertacao_8': {'trunk1_buoy': {'position': [44, 44], 'velocity': [0, 0]}, 'vegetation3_buoy': {'position': [200, 200], 'velocity': [0, 0]}},
    'sceinario_dissertacao_9': {'trunk1_buoy': {'position': [60, 60], 'velocity': [-0.85, -0.85]}},
    'sceinario_dissertacao_10': {'trunk1_buoy': {'position': [50, 0], 'velocity': [0, 0.85]}},
    'sceinario_dissertacao_11': {'trunk1_buoy': {'position': [50, 0], 'velocity': [0, 0.95]}},
    'sceinario_dissertacao_12': {'trunk1_buoy': {'position': [0, 50], 'velocity': [0.85, 0]}},
    'sceinario_dissertacao_13': {'trunk1_buoy': {'position': [0, 50], 'velocity': [0.95, -0.5]}},
    'sceinario_dissertacao_14': {'trunk1_buoy': {'position': [-10, -10], 'velocity': [1.4, 1.4]}},

    'sceinario_dissertacao_15': {'vegetation3_buoy': {'position': [44, 44], 'velocity': [0, 0]}, 'trunk1_buoy': {'position': [200, 200], 'velocity': [0, 0]}},
    'sceinario_dissertacao_16': {'vegetation3_buoy': {'position': [60, 60], 'velocity': [-0.85, -0.85]}},
    'sceinario_dissertacao_17': {'vegetation3_buoy': {'position': [50, 0], 'velocity': [0, 0.85]}},
    'sceinario_dissertacao_18': {'vegetation3_buoy': {'position': [50, 0], 'velocity': [0, 0.95]}},
    'sceinario_dissertacao_19': {'vegetation3_buoy': {'position': [0, 50], 'velocity': [0.85, 0]}},
    'sceinario_dissertacao_20': {'vegetation3_buoy': {'position': [0, 50], 'velocity': [0.95, -0.2]}},
    'sceinario_dissertacao_21': {'vegetation3_buoy': {'position': [-10, -10], 'velocity': [1.1, 1.1]}},
    
    #artigo:
    'scenario_from_table': {
        # Obstacles as per the table
        'vegetation3_buoy': {'position': [7.8, 2.2], 'velocity': [0, 0]},
        'vegetation3_buoy_clone': {'position': [6.8, 4.9], 'velocity': [0, 0]},
        'vegetation3_buoy_clone_clone': {'position': [7.0, 0], 'velocity': [-8, 8]},
        'vegetation3_buoy_clone_clone_clone': {'position': [7.5, 7], 'velocity': [-8, -8]},
        'branche3_buoy': {'position': [6, 8], 'velocity': [0, -3]},
        'vegetation3_buoy_trunk1_buoy': {'position': [4, 8], 'velocity': [2.8, -1.6]},
    },

    #artigo translated:
    'scenario_from_table_converted': {
        # Obstacles as per the table
        'vegetation3_buoy': {'position': [54.6, 15.4], 'velocity': [0, 0]},
        'vegetation3_buoy_clone': {'position': [47.6, 34.3], 'velocity': [0, 0]},
        'vegetation3_buoy_clone_clone': {'position': [49.0, 0], 'velocity': [-0.8, 0.8]},
        'vegetation3_buoy_clone_clone_clone': {'position': [52.5, 49.0], 'velocity': [-0.8, -0.8]},
        'branche3_buoy': {'position': [32.0, 46.0], 'velocity': [0, -0.5]},
        'trunk1_buoy': {'position': [25.0, 50.0], 'velocity': [0.2, -0.2]},
    },
    'scenario_from_table_converted_inverted': {
        # Obstáculos com posições invertidas para saída em (70,70) e chegada em (0,0)
        'vegetation3_buoy': {'position': [15.4, 54.6],'velocity': [0, 0]},
        'vegetation3_buoy_clone': {'position': [22.4, 35.7],'velocity': [0, 0]},
        'vegetation3_buoy_clone_clone': {'position': [21.0, 70],'velocity': [1, -1]},
        'vegetation3_buoy_clone_clone_clone': {'position': [17.5, 21.0],'velocity': [1, 1]},
        'branche3_buoy': {'position': [28.0, 14.0],'velocity': [0, 1.2]},
        'trunk1_buoy': {'position': [10.0, 14.0],'velocity': [-0.1, 0.1]},
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
    'scenario_teste_14': {'vegetation3_buoy': {'position': [-6.94, -39.39], 'velocity': [1.6, 1.1]}},  # colisão
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

    'scenario_teste_rotate_1': {'vegetation3_buoy': {'position': [25.73, 30.67], 'velocity': [-0.1061, -0.1768]}},
    'scenario_teste_rotate_2': {'vegetation3_buoy': {'position': [25.73, 30.67], 'velocity': [-0.1768, -0.1061]}},
    'scenario_teste_rotate_3': {'vegetation3_buoy': {'position': [25.73, 30.67], 'velocity': [0.1768, 0.1061]}},
    'scenario_teste_rotate_4': {'vegetation3_buoy': {'position': [25.73, 30.67], 'velocity': [0.1061, 0.1768]}},
    'scenario_teste_rotate_5': {'vegetation3_buoy': {'position': [30.67, 25.73], 'velocity': [-0.1061, -0.1768]}},
    'scenario_teste_rotate_6': {'vegetation3_buoy': {'position': [30.67, 25.73], 'velocity': [-0.1768, -0.1061]}},
    'scenario_teste_rotate_7': {'vegetation3_buoy': {'position': [30.67, 25.73], 'velocity': [0.1768, 0.1061]}},
    'scenario_teste_rotate_8': {'vegetation3_buoy': {'position': [30.67, 25.73], 'velocity': [0.1061, 0.1768]}},
    'scenario_teste_rotate_9': {'vegetation3_buoy': {'position': [40.00, 0.00], 'velocity': [0.0354, 0.6717]}},
    'scenario_teste_rotate_10': {'vegetation3_buoy': {'position': [40.00, 0.00], 'velocity': [-0.3536, 1.0607]}},
    'scenario_teste_rotate_11': {'vegetation3_buoy': {'position': [0.00, 40.00], 'velocity': [0.8130, 0.0354]}},
    'scenario_teste_rotate_12': {'vegetation3_buoy': {'position': [0.00, 40.00], 'velocity': [0.9546, -0.1061]}},
    'scenario_teste_rotate_13': {'vegetation3_buoy': {'position': [22.96, -32.77], 'velocity': [0.1414, 1.6970]}},
    'scenario_teste_rotate_14': {'vegetation3_buoy': {'position': [22.96, -32.77], 'velocity': [0.3536, 1.9092]}},
    'scenario_teste_rotate_15': {'vegetation3_buoy': {'position': [-36.24, 16.90], 'velocity': [1.6970, 0.1414]}},
    'scenario_teste_rotate_16': {'vegetation3_buoy': {'position': [-36.24, 16.90], 'velocity': [1.8385, 0.4243]}},
    'scenario_teste_rotate_17': {'vegetation3_buoy': {'position': [-20.00, -34.64], 'velocity': [1.5556, 1.9799]}},
    'scenario_teste_rotate_18': {'vegetation3_buoy': {'position': [-20.00, -34.64], 'velocity': [1.2728, 2.2627]}},
    'scenario_teste_rotate_19': {'vegetation3_buoy': {'position': [-34.64, -20.00], 'velocity': [1.9799, 1.5556]}},
    'scenario_teste_rotate_20': {'vegetation3_buoy': {'position': [-34.64, -20.00], 'velocity': [2.2627, 1.2728]}},
    'scenario_teste_rotate_21': {'vegetation3_buoy': {'position': [9.64, 11.50], 'velocity': [-0.1061, -0.1768]}},
    'scenario_teste_rotate_22': {'vegetation3_buoy': {'position': [9.64, 11.50], 'velocity': [-0.1768, -0.1061]}},
    'scenario_teste_rotate_23': {'vegetation3_buoy': {'position': [9.64, 11.50], 'velocity': [0.1768, 0.1061]}},
    'scenario_teste_rotate_24': {'vegetation3_buoy': {'position': [9.64, 11.50], 'velocity': [0.1061, 0.1768]}},
    'scenario_teste_rotate_25': {'vegetation3_buoy': {'position': [11.49, 9.63], 'velocity': [-0.1061, -0.1768]}},
    'scenario_teste_rotate_26': {'vegetation3_buoy': {'position': [11.49, 9.63], 'velocity': [-0.1768, -0.1061]}},
    'scenario_teste_rotate_27': {'vegetation3_buoy': {'position': [11.49, 9.63], 'velocity': [0.1768, 0.1061]}},
    'scenario_teste_rotate_28': {'vegetation3_buoy': {'position': [11.49, 9.63], 'velocity': [0.1061, 0.1768]}},
    'scenario_teste_rotate_29': {'vegetation3_buoy': {'position': [15.00, 0.00], 'velocity': [0.0354, 0.6717]}},
    'scenario_teste_rotate_30': {'vegetation3_buoy': {'position': [15.00, 0.00], 'velocity': [-0.3536, 1.0607]}},
    'scenario_teste_rotate_31': {'vegetation3_buoy': {'position': [0.00, 15.00], 'velocity': [0.8130, 0.0354]}},
    'scenario_teste_rotate_32': {'vegetation3_buoy': {'position': [0.00, 15.00], 'velocity': [0.9546, -0.1061]}},
    'scenario_teste_rotate_33': {'vegetation3_buoy': {'position': [8.63, -12.30], 'velocity': [0.1414, 1.6970]}},
    'scenario_teste_rotate_34': {'vegetation3_buoy': {'position': [8.63, -12.30], 'velocity': [0.4243, 1.8385]}},
    'scenario_teste_rotate_35': {'vegetation3_buoy': {'position': [-13.61, 6.35], 'velocity': [1.6970, 0.1414]}},
    'scenario_teste_rotate_36': {'vegetation3_buoy': {'position': [-13.61, 6.35], 'velocity': [1.8385, 0.4243]}},
    'scenario_teste_rotate_37': {'vegetation3_buoy': {'position': [-7.50, -12.98], 'velocity': [1.5556, 1.9799]}},
    'scenario_teste_rotate_38': {'vegetation3_buoy': {'position': [-7.50, -12.98], 'velocity': [1.2728, 2.2627]}},
    'scenario_teste_rotate_39': {'vegetation3_buoy': {'position': [-12.98, -7.50], 'velocity': [1.9799, 1.5556]}},
    'scenario_teste_rotate_40': {'vegetation3_buoy': {'position': [-12.98, -7.50], 'velocity': [2.2627, 1.2728]}}
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
        self.rotation = 30
        # Obter o nome do cenário a partir dos parâmetros ROS
        self.scenario_name = rospy.set_param('~scenario', 'sceinario_dissertacao_5')

        # Obter o nome do cenário
        self.scenario_real_name = rospy.get_param('~scenario')
        
        if self.scenario_real_name == 'scenario_from_table_converted' or self.scenario_real_name == 'scenario_from_table_converted_inverted':
            self.change_velocity = True
        else:
            self.change_velocity = False
        
        # Inicializar change_velocity
        
        
        # Subscriber para o tópico /change_velocity
        rospy.Subscriber('/change_velocity', Bool, self.change_velocity_callback)

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

        # Retrieve the robot domain radius parameter
        self.robot_domain_radius = rospy.get_param('/apfm_obstacle_avoidance/robot_domain_radius', 1.4)
        rospy.loginfo(f"robot_domain_radius : {self.robot_domain_radius}")

        # Configurar as posições e velocidades iniciais com base no cenário
        self.set_initial_positions()


        # Subscritor para o tópico de estados dos modelos do Gazebo
        self.model_state_sub = rospy.Subscriber(TOPIC_SUB, ModelStates, self.model_states_callback)
        self.time = 8.0
        # Se change_velocity estiver ativado, configurar um timer para 30 segundos
        if self.change_velocity:
            rospy.Timer(rospy.Duration(self.time), self.rotate_obstacle_velocities, oneshot=True)#update_obstacle_velocities, oneshot=True)


    def change_velocity_callback(self, msg):
        """
        Callback para atualizar o valor de change_velocity com base no tópico /change_velocity.
        """
        self.change_velocity = msg.data
        if self.change_velocity:
            rospy.Timer(rospy.Duration(self.time), self.rotate_obstacle_velocities, oneshot=True)#update_obstacle_velocities, oneshot=True)
        
        rospy.loginfo(f"change_velocity atualizado para: {self.change_velocity}")

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
        success = set_robot_position_and_velocity(
            INITIAL_ROBOT_POSE,
            INITIAL_ROBOT_VELOCITY,
            INITIAL_ROBOT_ORIENTATION
        )
        if success:
            rospy.loginfo("Posição e orientação do robô definidas com sucesso.")
        else:
            rospy.logerr("Falha ao definir a posição e orientação do robô.")

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

    def update_obstacle_velocities(self, event):
        rospy.loginfo("Atualizando velocidades dos obstáculos para apontar para o barco + 1 metro à frente.")

        # Obter a posição e orientação atual do robô
        robot_pose = self.migbot.position
        robot_orientation = self.migbot.orientation

        # Converter quaternion para ângulo de yaw
        euler = euler_from_quaternion([
            robot_orientation.x,
            robot_orientation.y,
            robot_orientation.z,
            robot_orientation.w
        ])
        yaw = euler[2]

        # Calcular o ponto 1 metro à frente da direção do robô
        target_point_x = robot_pose.x + np.cos(yaw) * 5.0  # 5 metro à frente
        target_point_y = robot_pose.y + np.sin(yaw) * 5.0
        target_point = np.array([target_point_x, target_point_y])

        rospy.loginfo(f"Ponto de destino (1m à frente): ({target_point_x}, {target_point_y})")

        for ob in self.obstacles:
            obstacle_position = np.array([ob.position.x, ob.position.y])
            direction_vector = target_point - obstacle_position
            distance = np.linalg.norm(direction_vector)
            if distance == 0:
                rospy.logwarn(f"Obstáculo {ob.name} está exatamente no ponto de destino. Mantendo velocidade atual.")
                continue
            normalized_vector = direction_vector / distance  # Vetor unitário
            desired_speed = np.linalg.norm([ob.velocity.x, ob.velocity.y])  # Mantém a velocidade atual

            # Se a velocidade atual for zero, definir uma velocidade padrão
            if desired_speed == 0:
                desired_speed = 1.0  # Por exemplo, 1 m/s

            new_velocity = normalized_vector * desired_speed

            ob.velocity.x = new_velocity[0]
            ob.velocity.y = new_velocity[1]

            # Atualizar a velocidade do obstáculo no Gazebo
            set_obstacle_position_and_velocity(ob.name, [ob.position.x, ob.position.y], new_velocity)

            rospy.loginfo(f"Obstáculo {ob.name} atualizado para nova velocidade: ({new_velocity[0]:.2f}, {new_velocity[1]:.2f})")

        rospy.loginfo("Velocidades dos obstáculos atualizadas com sucesso.")

def set_robot_position_and_velocity(position, velocity, orientation_degrees):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state = ModelState()
        state.model_name = ROBOT_NAME
        state.pose.position.x = position[0]
        state.pose.position.y = position[1]
        state.twist.linear.x = velocity[0]
        state.twist.linear.y = velocity[1]
        state.twist.angular.x = velocity[0]
        state.twist.angular.y = velocity[1]
        
        # Converter a orientação de graus para radianos
        orientation_radians = np.deg2rad(orientation_degrees)
        
        # Converter o ângulo de Euler (yaw) para quaternion
        orientation_quat = quaternion_from_euler(0, 0, orientation_radians)
        
        # Definir a orientação na pose do robô
        state.pose.orientation = Quaternion(
            x=orientation_quat[0],
            y=orientation_quat[1],
            z=orientation_quat[2],
            w=orientation_quat[3]
        )

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
        state.twist.linear.x = 2*velocity[0]
        state.twist.linear.y = 2*velocity[1]
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