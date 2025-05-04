import argparse
from time import sleep

from as2_python_api.drone_interface import DroneInterface
import rclpy

TAKE_OFF_HEIGHT = 1.5  # Height in meters
TAKE_OFF_SPEED = 1.0  # Max speed in m/s
SLEEP_TIME = 0.5  # Sleep time between behaviors in seconds
SPEED = 1.0  # Max speed in m/s
HEIGHT = 3.0  # Height in meters
DIM = 2.0

PATH = [
    [2.0,  0.0, HEIGHT],
    [2.0, -2.0, HEIGHT],
    [0.0, -22.0, HEIGHT],
    [0.0,  0.0, HEIGHT]
]

LAND_SPEED = 0.5  # Max speed in m/s


def drone_start(drone_interface: DroneInterface) -> bool:

    # Arm
    print('Arm')
    success = drone_interface.arm()
    print(f'Arm success: {success}')

    # Offboard
    print('Offboard')
    success = drone_interface.offboard()
    print(f'Offboard success: {success}')
    
    # Take Off
    print('Take Off')
    success = drone_interface.takeoff(height=TAKE_OFF_HEIGHT, speed=TAKE_OFF_SPEED)
    print(f'Take Off success: {success}')

    return success


def drone_run(drone_interface: DroneInterface) -> bool:


    # Go to path with keep yaw
    for goal in PATH:
        print(f'Go to with keep yaw {goal}')
        success = drone_interface.go_to.go_to_point_path_facing(goal, speed=SPEED)
        print(f'Go to success: {success}')
        if not success:
            return success
        print('Go to done')
        sleep(SLEEP_TIME)

def drone_end(drone_interface: DroneInterface) -> bool:

    # Land
    print('Land')
    success = drone_interface.land(speed=LAND_SPEED)
    print(f'Land success: {success}')
    if not success:
        return success

    # Manual
    print('Manual')
    success = drone_interface.manual()
    print(f'Manual success: {success}')

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
                        default=False,
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
