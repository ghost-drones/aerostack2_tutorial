#!/usr/bin/env python3

__author__ = 'Lucca Gandra'

import argparse
from time import sleep

from as2_python_api.drone_interface import DroneInterface
import rclpy
import numpy as np
from math import cos, sin, pi
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

TAKE_OFF_HEIGHT = 3.0  # Height in meters
TAKE_OFF_SPEED = 1.0  # Max speed in m/s
SLEEP_TIME = 0.5  # Sleep time between behaviors in seconds
SPEED = 1.0  # Max speed in m/s
HEIGHT = 1.0  # Height in meters
LAND_SPEED = 0.5  # Max speed in m/s

def generate_circle_trajectory(radius, iterations, height):

    path_msg = Path()
    path_msg.header.frame_id = "earth"

    for i in range(iterations):
        pose = PoseStamped()
        pose.header.frame_id = "earth"

        angle = 2 * pi * (i + 1) / iterations
        pose.pose.position.x = radius * cos(angle)
        pose.pose.position.y = radius * sin(angle)
        pose.pose.position.z = height

        path_msg.poses.append(pose)

    return path_msg

def drone_start(drone_interface: DroneInterface) -> bool:

    print('Start mission')

    success = drone_interface.arm()
    success = drone_interface.offboard()
    success = drone_interface.takeoff(height=TAKE_OFF_HEIGHT, speed=TAKE_OFF_SPEED)

    return success


def drone_run(drone_interface: DroneInterface) -> bool:

    print('Run mission')

    goal = [2.0, 0.0, 3.0]
    final_goal = [0.0, 0.0, 3.0]

    success = drone_interface.go_to.go_to_point_path_facing(goal, speed=SPEED)

    if not success:
        return success
    
    circle_path = generate_circle_trajectory(2.0, 10, 3.0)

    success = drone_interface.follow_path.follow_path_with_path_facing(
            circle_path, speed=10*SPEED)
    
    if not success:
        return success

    success = drone_interface.go_to.go_to_point_path_facing(final_goal, speed=SPEED)
    
    if not success:
        return success

def drone_end(drone_interface: DroneInterface) -> bool:

    print('End mission')

    success = drone_interface.land(speed=LAND_SPEED)

    if not success:
        return success

    success = drone_interface.manual()

    return success


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Single drone mission')

    parser.add_argument('-n', '--namespace',
                        type=str,
                        default='drone0',
                        help='ID of the drone to be used in the mission')
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        default=False,
                        help='Enable verbose output')
    parser.add_argument('-s', '--use_sim_time',
                        action='store_true',
                        default=True,
                        help='Use simulation time')

    args = parser.parse_args()
    drone_namespace = args.namespace
    verbosity = args.verbose
    use_sim_time = args.use_sim_time

    print(f'Running mission for drone {drone_namespace}')

    rclpy.init()

    uav = DroneInterface(
        drone_id=drone_namespace,
        use_sim_time=use_sim_time,
        verbose=verbosity)

    success = drone_start(uav)
    if success:
        success = drone_run(uav)
    success = drone_end(uav)

    uav.shutdown()
    rclpy.shutdown()
    print('Clean exit')
    exit(0)
