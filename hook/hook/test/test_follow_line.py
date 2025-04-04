#!/usr/bin/env python3

import rclpy
import time

from yasmin import StateMachine
from yasmin_ros.yasmin_node import YasminNode
from yasmin_viewer import YasminViewerPub
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

# Import your specific state machine and the drone interface
from hook.follow_line.follow_line_sm import FollowStateMachine
from mirela_sdk.control.mavros.mavros_api import MavDrone


def main() -> None:
    print("--- Testing FollowStateMachine ---")

    rclpy.init()

    # Create a node instance using YasminNode
    node = YasminNode.get_instance()

    try:
        # Instantiate the drone interface
        drone = MavDrone(node, False)

        # Instantiate the state machine to test
        follow_sm = FollowStateMachine(drone)

        # Optional: Publish the state machine structure for visualization
        YasminViewerPub("follow_line_test_sm", follow_sm)
        print("YasminViewerPub initialized. Check RViz or other visualizers.")

        # Execute the state machine
        print("Executing FollowStateMachine...")
        # Give a moment for ROS connections to establish
        time.sleep(2)
        outcome = follow_sm()

        # Print the final outcome
        print(f"FollowStateMachine finished with outcome: {outcome}")

    except Exception as e:
        node.get_logger().error(
            f"An error occurred during state machine execution: {str(e)}"
        )
        # Optional: Print traceback for detailed debugging
        import traceback

        traceback.print_exc()
    finally:
        # Ensure ROS 2 is shut down cleanly
        print("Shutting down ROS 2...")
        rclpy.shutdown()
        print("--- Test Finished ---")


if __name__ == "__main__":
    main()
