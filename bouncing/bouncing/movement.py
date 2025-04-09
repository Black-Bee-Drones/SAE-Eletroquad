import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from typing import Optional
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


class MovementNode(Node):
    def __init__(self) -> None:
        super().__init__('movement_node')

        self.pid_x = PID(kp=0.1, ki=0.0, kd=0.01)
        self.pid_y = PID(kp=0.1, ki=0.0, kd=0.01)

        self.status_pub = self.create_publisher(String, "/movement_status", 10)

        self.create_subscription(Float32MultiArray, "/figure_error", self.error_cb, 10)
        self.create_subscription(String, "/mission_cmd", self.cmd_cb, 10)

        self.error_threshold: float = 15.0  # pixels
        self.last_time: float = self.get_clock().now().nanoseconds / 1e9
        self.landed: bool = False

        self.drone: MavDrone = MavDrone(self, False)

    def cmd_cb(self, msg: String) -> None:
        command: str = msg.data
        if command == "takeoff":
            self.get_logger().info("[Movement] Taking off...")
            self.drone.arm_takeoff(altitude=1.0)
            time.sleep(5)
            self.landed = False
        else:
            self.get_logger().warning(f"[Movement] Unknown command: {command}")

    def error_cb(self, msg: Float32MultiArray) -> None:
        if self.landed or len(msg.data) < 2:
            return

        dx: float = msg.data[0]
        dy: float = msg.data[1]

        current_time: float = self.get_clock().now().nanoseconds / 1e9
        dt: float = current_time - self.last_time
        self.last_time = current_time

        output_x: float = self.pid_x.compute(dx, dt)
        output_y: float = self.pid_y.compute(dy, dt)

        # implementar movimentação aqui! ATENÇÃO ao sinal do PID!!!
        self.get_logger().info(f"[Movement] dx={dx:.2f}, dy={dy:.2f}, output_x={output_x:.2f}, output_y={output_y:.2f}")

        if abs(dx) < self.error_threshold and abs(dy) < self.error_threshold:
            self.get_logger().info("[Movement] Aligned! Starting to land...")

            land_msg: String = String()
            land_msg.data = "land"
            self.drone.land()
            time.sleep(5)
            self.get_logger().info("[Movement] Landed.")

            self.landed = True

            status_msg: String = String()
            status_msg.data = "landed"
            self.status_pub.publish(status_msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MovementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
