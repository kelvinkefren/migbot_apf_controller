#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_scenario_A3_U_trap.py

Cenário fixo (v2 NO-CLONES): A3_U_trap
"""
import numpy as np
import rospy
from gazebo_scenario_v2_lib import GazeboScenarioV2Runner, unit, rot90_ccw

def build_cfg(start, goal, dm_ref):
    start = np.array(start, dtype=float)
    goal  = np.array(goal, dtype=float)
    u = unit(goal - start)
    n = rot90_ccw(u)

    def along(t):
        return start + t*(goal - start)

    def W(f):
        return float(f * dm_ref)

    return dict(
        robot=dict(pos=start, vel=[0.0,0.0], yaw_deg=45.0),
        obstacles={
        'vegetation3_buoy': dict(pos=along(0.50) + 0.7*dm_ref*n, vel=[0.0,0.0]),
        'vegetation1_buoy': dict(pos=along(0.58) + 0.7*dm_ref*n, vel=[0.0,0.0]),
        'branche3_buoy': dict(pos=along(0.58) + 0.0*dm_ref*n, vel=[0.0,0.0]),
    },
        walls=dict(enabled=True, kind='straight', width=W(3.2), pieces=3),
        events=[]
    )

if __name__ == '__main__':
    try:
        GazeboScenarioV2Runner('A3_U_trap', build_cfg).spin()
    except rospy.ROSInterruptException:
        pass
