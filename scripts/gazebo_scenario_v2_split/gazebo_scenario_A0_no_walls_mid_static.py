#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_scenario_A0_no_walls_mid_static.py

Cenário fixo (v2 NO-CLONES): sem paredes virtuais
- 1 obstáculo estático no meio do caminho (t = 0.50)
"""
import numpy as np
import rospy
from gazebo_scenario_v2_lib import GazeboScenarioV2Runner, unit, rot90_ccw

def build_cfg(start, goal, dm_ref):
    start = np.array(start, dtype=float)
    goal  = np.array(goal, dtype=float)
    u = unit(goal - start)
    n = rot90_ccw(u)  # (não é necessário aqui, mas mantém o padrão)

    def along(t):
        return start + t * (goal - start)

    return dict(
        robot=dict(pos=start, vel=[0.0, 0.0], yaw_deg=45.0),

        # obstáculo estático no meio do caminho
        obstacles={
            'vegetation3_buoy': dict(pos=along(0.50), vel=[0.0, 0.0]),
        },

        # paredes DESABILITADAS
        walls=dict(enabled=False, kind='straight', width=float(0.0), pieces=1),

        events=[]
    )

if __name__ == '__main__':
    try:
        GazeboScenarioV2Runner('A0_no_walls_mid_static', build_cfg).spin()
    except rospy.ROSInterruptException:
        pass

