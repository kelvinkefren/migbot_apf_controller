#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_scenario_B2_crossing.py

Cenário fixo (v2 NO-CLONES): B2_crossing
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
        'vegetation3_buoy': dict(pos=along(0.55) - 1.4*dm_ref*n, vel=(0.9*n).tolist()),
    },
        walls=dict(enabled=True, kind='straight', width=W(3.0), pieces=2),
        events=[]
    )

if __name__ == '__main__':
    try:
        GazeboScenarioV2Runner('B2_crossing', build_cfg).spin()
    except rospy.ROSInterruptException:
        pass
