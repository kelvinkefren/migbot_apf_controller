#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
import threading

class ScenarioManager:
    def __init__(self):
        # Inicializa o nó ROS
        rospy.init_node('scenario_manager', anonymous=True)

        self.scenario_pub = rospy.Publisher('/current_scenario', String, queue_size=10)
        self.change_velocity_pub = rospy.Publisher('/change_velocity', Bool, queue_size=10)

        # Parâmetros iniciais
        self.current_scenario = 14
        self.max_scenarios = 21

        # Flags de controle
        self.scenario_changing = False
        self.in_collision = False
        self.lock = threading.Lock()

        self.simulador=2
        # Define o cenário inicial
        self.set_scenario_parameter(self.current_scenario)
        rospy.loginfo(f"Scenario Manager iniciado com {self.get_current_scenario_name()}")

        # Subscribers
        self.reached_goal_sub = rospy.Subscriber('/apfm/reached_goal', Bool, self.goal_callback)
        self.colisao_sub = rospy.Subscriber('/obstacle_avoidance/collision', Bool, self.colisao_callback)

    def get_current_scenario_name(self):
        """
        Retorna o nome do cenário atual baseado no número do cenário.
        """
        if self.simulador==1:
            scenario_name = f"scenario_teste_{self.current_scenario}"
            self.publish_change_velocity(False)
        
        if self.simulador==2:
            if self.current_scenario > 14:
                self.publish_change_velocity(True)
            else:                
                self.publish_change_velocity(False)
            scenario_name = f"sceinario_dissertacao_{self.current_scenario}"

        if self.simulador==3:
            scenario_name = f"scenario_teste_rotate_{self.current_scenario}"
            self.publish_change_velocity(False)
        


        # Publicar o nome do cenário
        self.scenario_pub.publish(scenario_name)
        return scenario_name


    def publish_change_velocity(self, value):
        """
        Publica o valor de change_velocity no tópico /change_velocity.
        """
        msg = Bool()
        msg.data = value
        self.change_velocity_pub.publish(msg)

    def set_scenario_parameter(self, scenario_number):
        """
        Define o parâmetro ROS para o cenário atual.
        """
        scenario_name = self.get_current_scenario_name()
        rospy.set_param('/gazebo_scenario/scenario', scenario_name)
        rospy.loginfo(f"Parâmetro '/gazebo_scenario/scenario' definido para '{scenario_name}'")

    def goal_callback(self, msg):
        """
        Callback chamado quando uma mensagem é recebida no tópico /apfm/reached_goal.
        Se msg.data for True e não houver uma mudança de cenário em progresso, incrementa o cenário e define o novo parâmetro.
        """
        if msg.data:
            with self.lock:
                if not self.scenario_changing:
                    rospy.loginfo("Objetivo alcançado. Preparando para avançar para o próximo cenário.")
                    threading.Thread(target=self.advance_scenario).start()

    def colisao_callback(self, msg):
        """
        Callback chamado quando uma mensagem é recebida no tópico /obstacle_avoidance/collision.
        Trata colisões detectadas.
        """
        with self.lock:
            if msg.data:
                if not self.in_collision:
                    rospy.logwarn("Colisão detectada! Iniciando tratamento da colisão.")
                    self.in_collision = True
                    if not self.scenario_changing:
                        threading.Thread(target=self.handle_collision).start()
            else:
                if self.in_collision:
                    rospy.loginfo("Colisão resolvida.")
                    self.in_collision = False

    def handle_collision(self):
        """
        Trata a colisão detectada. Avança para o próximo cenário.
        """
        with self.lock:
            self.scenario_changing = True
        rospy.loginfo("Tratando colisão. Avançando para o próximo cenário.")
        
        if self.current_scenario < self.max_scenarios:
            self.current_scenario += 1
            self.set_scenario_parameter(self.current_scenario)
            rospy.loginfo(f"Cenário avançado para {self.get_current_scenario_name()} após colisão.")
        else:
            rospy.loginfo("Todos os cenários foram executados. Encerrando o Scenario Manager.")
            rospy.signal_shutdown("Fim dos cenários.")
        
        rospy.sleep(3)  # Aguarda 3 segundos para permitir o carregamento do cenário
        
        with self.lock:
            self.scenario_changing = False

    def advance_scenario(self):
        """
        Avança para o próximo cenário, definindo o novo parâmetro ROS.
        Aguarda 3 segundos após definir o parâmetro para permitir o carregamento do novo cenário.
        """
        with self.lock:
            self.scenario_changing = True
        if self.current_scenario < self.max_scenarios:
            self.current_scenario += 1
            self.set_scenario_parameter(self.current_scenario)
            rospy.loginfo(f"Avançado para {self.get_current_scenario_name()}")
            rospy.sleep(3)  # Aguarda 3 segundos para permitir o carregamento do novo cenário
        else:
            rospy.loginfo("Todos os cenários foram executados. Encerrando o Scenario Manager.")
            rospy.signal_shutdown("Fim dos cenários.")
        with self.lock:
            self.scenario_changing = False

if __name__ == '__main__':
    try:
        manager = ScenarioManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
