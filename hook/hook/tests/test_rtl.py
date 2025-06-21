#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from mirela_sdk.control.mavros.mavros_api import MavDrone


class TestRTLNode(Node):
    def __init__(self):
        super().__init__('test_rtl_node')
        self.get_logger().info('Starting RTL test node...')
        self.drone = MavDrone(self)
        self.test_sequence()

    def test_sequence(self):
        self.get_logger().info('RTL Test Sequence Starting')

        self.get_logger().info('Arming and taking off')
        self.drone.set_home(current_gps=True)
        self.drone.arm_takeoff(3.0)  # Takeoff to 3 meters
        self.get_logger().info('Waiting to reach takeoff altitude')
        time.sleep(3.0)
            
        self.get_logger().info('Moving forward')
        # Set velocity in x direction (forward) for 5 seconds
        self.drone.offboard_velocity_timer(
            linear_x=1.0,  # 1 m/s forward
            linear_y=0.0,
            linear_z=0.0,
            angular_z=0.0,
            ground_reference=False,
            pub_rate=30,
            time=3
        )
        
        # Step 3: Move right
        self.get_logger().info('Moving right')
        # Set velocity in y direction (right) for 5 seconds
        self.drone.offboard_velocity_timer(
            linear_x=0.0,
            linear_y=1.0,  # 1 m/s right
            linear_z=0.0,
            angular_z=0.0,
            ground_reference=False,
            pub_rate=30,
            time=3
        )
         
        # Step 4: Execute RTL
        self.get_logger().info('Executing RTL')
        self.drone.rtl(rtl_alt=5)  # RTL altitude of 5 meters
            
        # Step 5: Test complete
        self.get_logger().info('RTL Test Complete')


def main(args=None):
    rclpy.init(args=args)
    node = TestRTLNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
