# migbot Artificial Potential Field Modified Controller

## Overview

This package is designed to receive Force from the Dynamic Obstacle Avoidance Package and calculate the /Wrench for the migbot robot.

## Topic input:

    /apfm/total_force: Vector3 (Total force)

## Topic output:

    /APFM/Wrench

## Params

Before running the nodes, you need to define the following parameters. Below are the parameters with their default values:

```yaml
max_wrench: 0.2
MAX_TORQUE_WRENCH: 0.1
MIN_TORQUE_WRENCH: -0.1
MAX_LINEAR_SPEED: 1.0
TOLERANCE: 5.0
```

## Debugging force

Will print a 2D graph with robot and obstacle position, and the force vector in green, the other is the velocity.

    rosrun migbot_apf_controller debug_plot

## SETUP

    rosrun migbot_apf_controller gazebo_scenario

The code will configure the migbot simulation. It is necessary to edit the code to change obstacles location.
OBSTACLE_NAMES = ['vegetation1_buoy','vegetation2_buoy','vegetation3_buoy','branche3_buoy']  # obstacle names from gazebo
SCENARIOS just have one option, you need to write the position and velocity that you want to put the obstacle.
INITIAL_ROBOT_POSE -> desired robot position to start simulation
INITIAL_ROBOT_VELOCITY -> just put in zero

    -TODO params

Don't need to close the others nodes to start this one.
