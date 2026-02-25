#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_scenario_C4_curve_bottleneck_encounter.py

Cenário fixo (v2 NO-CLONES): C4_curve_bottleneck_encounter
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
        'vegetation1_buoy': dict(pos=along(0.60) + 0.65*dm_ref*n, vel=[0.0,0.0]),
        'branche3_buoy': dict(pos=along(0.60) - 0.65*dm_ref*n, vel=[0.0,0.0]),
        'vegetation3_buoy': dict(pos=along(0.85), vel=(-0.70*u).tolist()),
    },
        walls=dict(enabled=True, kind='s_curve', width=W(3.0), pieces=3),
        events=[
        dict(t=7.0, type='set_velocity', target='vegetation3_buoy', vel=(-0.95*u).tolist()),
    ]
    )

if __name__ == '__main__':
    try:
        GazeboScenarioV2Runner('C4_curve_bottleneck_encounter', build_cfg).spin()
    except rospy.ROSInterruptException:
        pass
