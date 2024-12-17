#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

class ScenarioManager:
    def __init__(self):
        # Inicializa o nó ROS
        rospy.init_node('scenario_manager', anonymous=True)

        # Parâmetros iniciais
        self.current_scenario = 1
        self.max_scenarios = 40

        # Define o cenário inicial
        self.set_scenario_parameter(self.current_scenario)

        rospy.loginfo(f"Scenario Manager iniciado com {self.get_current_scenario_name()}")

        # Subscriber para o tópico /apfm/reached_goal
        self.reached_goal_sub = rospy.Subscriber('/apfm/reached_goal', Bool, self.goal_callback)

    def get_current_scenario_name(self):
        """
        Retorna o nome do cenário atual baseado no número do cenário.
        """
        return f"scenario_teste_{self.current_scenario}"

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
        Se msg.data for True, incrementa o cenário e define o novo parâmetro.
        """
        if msg.data:
            rospy.loginfo("Objetivo alcançado. Preparando para avançar para o próximo cenário.")
            self.advance_scenario()

    def advance_scenario(self):
        """
        Avança para o próximo cenário, definindo o novo parâmetro ROS.
        Aguarda 3 segundos após definir o parâmetro para permitir o carregamento do novo cenário.
        """
        if self.current_scenario < self.max_scenarios:
            self.current_scenario += 1
            self.set_scenario_parameter(self.current_scenario)
            rospy.loginfo(f"Avançado para {self.get_current_scenario_name()}")
            
            # Aguarda 3 segundos para permitir o carregamento do novo cenário
            rospy.sleep(3)
        else:
            rospy.loginfo("Todos os cenários foram executados. Encerrando o Scenario Manager.")
            rospy.signal_shutdown("Fim dos cenários.")

if __name__ == '__main__':
    try:
        manager = ScenarioManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
