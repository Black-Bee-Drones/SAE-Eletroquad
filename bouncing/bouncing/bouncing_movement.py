import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from rclpy.qos import qos_profile_sensor_data
from typing import Optional, Tuple
from mirela_sdk.control.mavros.mavros_api import MavDrone


class PID:
    def __init__(self, kp: float, ki: float, kd: float) -> None:
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd

        self.integral: float = 0.0
        self.prev_error: Optional[float] = None

    def compute(self, error: float, dt: float) -> float:
        self.integral += error * dt
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    
    def restart(self) -> None:
        self.integral = 0.0
        self.prev_error = None


class MovementNode(Node):
    """
    ROS2 Node that controls drone movement using visual feedback and PID controllers.

    This node:
    - Receives mission commands via the "/mission_cmd" topic (e.g., "takeoff")
    - Receives visual alignment error (dx, dy) via the "/figure_error" topic
    - Applies PID controllers for visual alignment correction
    - Initiates automatic landing when alignment is within a threshold
    - Publishes landing status to the "/movement_status" topic
    """

    def __init__(
            self, 
            p1_lat: float, p1_lon: float,
            p2_lat: float, p2_lon: float,
            p3_lat: float, p3_lon: float,
            p4_lat: float, p4_lon: float
        ) -> None:
        super().__init__('movement_node')

        self.drone: MavDrone = MavDrone(self, False)

        # Define corners of the mapped area
        self.corner_top_left: Tuple[float, float] = (p1_lat, p1_lon)     
        self.corner_top_right: Tuple[float, float] = (p2_lat, p2_lon)    
        self.corner_bottom_left: Tuple[float, float] = (p3_lat, p3_lon)  
        self.corner_bottom_right: Tuple[float, float] = (p4_lat, p4_lon) 

        # Search waypoints (interpolated between corners)
        self.search_point_left: Tuple[float, float]
        self.search_point_middle: Tuple[float, float]
        self.search_point_right: Tuple[float, float]

        self.points_calculation()

        self.error_threshold: float = 5.0  # pixels
        self.last_time: float = 0.0
        self.landed: bool = False

        self.pid_x = PID(kp=0.1, ki=0.0, kd=0.01)
        self.pid_y = PID(kp=0.1, ki=0.0, kd=0.01)

        self.status_pub = self.create_publisher(String, "/movement_status", 10)

        self.create_subscription(Float32MultiArray, "/figure_error", self.error_cb, qos_profile_sensor_data)
        self.create_subscription(String, "/mission_cmd", self.cmd_cb, 10)

    def cmd_cb(self, msg: String) -> None:
        """
        Callback for receiving mission commands (e.g., "takeoff").

        Args:
            msg (String): The message received from the "/mission_cmd" topic.
        """
        command: str = msg.data
        if command == "takeoff":
            self.get_logger().info("[Movement] Taking off...")
            self.drone.arm_takeoff(1.0)
            time.sleep(5)
            self.landed = False
        else:
            self.get_logger().warning(f"[Movement] Unknown command: {command}")

    def error_cb(self, msg: Float32MultiArray) -> None:
        """
        Callback for receiving alignment errors (dx, dy) from visual feedback.

        Args:
            msg (Float32MultiArray): Error in pixels received from the vision system.
        """
        if self.landed:
            self.get_logger().debug("[MovementNode] Drone already landed. Ignoring visual error.")
            return
        
        dx: float = msg.data[0]
        dy: float = msg.data[1]

        current_time: float = self.get_clock().now().nanoseconds / 1e9
        dt: float = current_time - self.last_time
        self.last_time = current_time

        output_x: float = self.pid_x.compute(dx, dt)
        output_y: float = self.pid_y.compute(dy, dt)

        # TODO: Implement movement logic here using output_x and output_y
        self.get_logger().debug(
            f"[PID] Input dx={dx:.2f}, dy={dy:.2f} | Output vx={output_x:.2f}, vy={output_y:.2f} | dt={dt:.3f}"
        )

        if abs(dx) < self.error_threshold and abs(dy) < self.error_threshold:
            self.get_logger().info("[Movement] Aligned! Starting to land...")

            self.pid_x.restart()
            self.pid_y.restart()

            self.drone.land()
            time.sleep(5)
            self.get_logger().info("[Movement] Landed.")

            self.landed = True

            status_msg: String = String()
            status_msg.data = "landed"
            self.status_pub.publish(status_msg)

    def points_calculation(self) -> None:
        """
        Computes intermediate search points across the mapped region using geodesic interpolation.
        The points are evenly spaced along the center axis of the area, and divide the area in 3 sections, which will map the entire area.
        """
        midpoint_top = self.drone.gps_controller.interp_geo(self.corner_top_left, self.corner_top_right, 0.5)
        midpoint_bottom = self.drone.gps_controller.interp_geo(self.corner_top_right, self.corner_bottom_right, 0.5)

        self.search_point_left = self.interp_geo(midpoint_top, midpoint_bottom, 1/6)
        self.search_point_middle = self.interp_geo(midpoint_top, midpoint_bottom, 3/6)
        self.search_point_right = self.interp_geo(midpoint_top, midpoint_bottom, 5/6)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MovementNode(0, 0, 0, 0, 0, 0, 0, 0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
