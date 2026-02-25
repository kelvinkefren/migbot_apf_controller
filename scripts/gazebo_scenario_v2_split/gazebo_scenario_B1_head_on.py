#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_scenario_B1_head_on.py

Cenário fixo (v2 NO-CLONES): B1_head_on
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
        'trunk1_buoy': dict(pos=goal, vel=[-0.85, -0.85]),
    },
        walls=dict(enabled=True, kind='straight', width=W(3.2), pieces=2),
        events=[
        dict(t=8.0, type='scale_velocity', target='trunk1_buoy', scale=1.4),
    ]
    )

if __name__ == '__main__':
    try:
        GazeboScenarioV2Runner('B1_head_on', build_cfg).spin()
    except rospy.ROSInterruptException:
        pass
