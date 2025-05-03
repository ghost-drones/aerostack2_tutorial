#!/usr/bin/env python3

__author__ = 'Lucca Gandra'

import argparse
from time import sleep

from as2_python_api.drone_interface import DroneInterface
import rclpy

TAKE_OFF_HEIGHT = 3.0  # Height in meters
TAKE_OFF_SPEED = 1.0  # Max speed in m/s
SLEEP_TIME = 0.5  # Sleep time between behaviors in seconds
SPEED = 1.0  # Max speed in m/s
HEIGHT = 1.0  # Height in meters
LAND_SPEED = 0.5  # Max speed in m/s

def drone_start(drone_interface: DroneInterface) -> bool:

    print('Start mission')

    success = drone_interface.arm()
    success = drone_interface.offboard()
    success = drone_interface.takeoff(height=TAKE_OFF_HEIGHT, speed=TAKE_OFF_SPEED)

    return success


def drone_run(drone_interface: DroneInterface) -> bool:

    print('Run mission')

    goal = [2.0, 0.0, 3.0]
    sucess = drone_interface.go_to.go_to_point(goal, speed=SPEED)

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
                        default='x500_px4',
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
