#!/usr/bin/env python3

import os
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter
from typing import Tuple, List, Optional
from std_msgs.msg import Float64, Float32MultiArray
from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.image_processing.camera.image_calculus import ImageCalculus

class BouncingNodeV2(Node):
    """
    ROS 2 node that manages a drone's search and detection routine for geometric figures.
    Uses a PID controller for precise positioning and the YOLO inference node for detection.
    """

    # Mission constants
    TAKEOFF_ALTITUDE = 6.5  # meters
    FINAL_ALTITUDE = 2.0    # meters
    DESCENT_SPEED = 0.3     # m/s (negative for descent)
    CAMERA_OFFSET = 0.12    # meters (distance from camera to drone center)
    CAMERA_FOV = 43.3       # degrees (vertical FOV)
    
    # PID Control constants
    CENTERING_THRESHOLD = 0.1  # Control effort below this value is considered centered
    MAX_VELOCITY = 0.5         # Maximum velocity in m/s
    
    # Figure definitions
    FIGURE_MAP = {
        "circle": 0,
        "square": 1,
        "triangle": 2,
        "hexagon": 3,
        "pentagon": 4,
        "star": 5,
        "cross": 6,
        "house": 7
    }

    def __init__(
            self, 
            figure: str,
            p1_lat: float, p1_lon: float,
            p2_lat: float, p2_lon: float,
            p3_lat: float, p3_lon: float,
            p4_lat: float, p4_lon: float
        ) -> None:
        """
        Initialize the BouncingNodeV2 with target figure and GPS search area.
        
        Args:
            figure (str): Target figure name ("circle", "square", etc.)
            p1_lat, p1_lon, ..., p4_lon: GPS coordinates of search area corners
        """
        super().__init__('bouncing_node_v2')
        
        # Initialize drone
        self.drone = MavDrone(self, False)
        
        # Get figure class ID
        self.figure_class = self.FIGURE_MAP.get(figure.lower())
        if self.figure_class is None:
            self.get_logger().error(f"Invalid figure '{figure}'. Valid options: {list(self.FIGURE_MAP.keys())}")
            raise ValueError(f"Invalid figure '{figure}'")
        
        self.get_logger().info(f"Target figure: {figure} (class {self.figure_class})")
        
        # Store search area
        self.corner_top_left = (p1_lat, p1_lon)
        self.corner_top_right = (p2_lat, p2_lon)
        self.corner_bottom_left = (p3_lat, p3_lon)
        self.corner_bottom_right = (p4_lat, p4_lon)
        
        # Mission state
        self.mission_complete = False
        self.figure_detected = False
        self.centering_complete = False
        self.current_altitude = 0.0
        
        # Detection results
        self.detected_objects = []  # List of (class_id, cx, cy, width, height)
        self.last_detection_time = self.get_clock().now()
        
        # Calculate search waypoints
        self.search_points = self._calculate_search_points()
        
        # QoS profile for detection topics
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to YOLO detection results (single topic with all info)
        self.detections_sub = self.create_subscription(
            Float32MultiArray,
            '/yolo_detections',
            self._detections_callback,
            qos_profile
        )
        
        # Subscribe to PID controller outputs
        self.pid_x_effort_sub = self.create_subscription(
            Float64,
            '/pid_controller_x/control_effort',
            self._pid_x_callback,
            10
        )
        
        self.pid_y_effort_sub = self.create_subscription(
            Float64,
            '/pid_controller_y/control_effort',
            self._pid_y_callback,
            10
        )
        
        # PID setpoint and state publishers
        self.pid_x_setpoint_pub = self.create_publisher(Float64, '/pid_controller_x/setpoint', 10)
        self.pid_y_setpoint_pub = self.create_publisher(Float64, '/pid_controller_y/setpoint', 10)
        self.pid_x_state_pub = self.create_publisher(Float64, '/pid_controller_x/state', 10)
        self.pid_y_state_pub = self.create_publisher(Float64, '/pid_controller_y/state', 10)
        
        # Control values
        self.pid_x_effort = 0.0
        self.pid_y_effort = 0.0
        
        # Set image dimensions
        self.image_width = 640
        self.image_height = 480
        
        # Create timer for main control loop
        self.control_timer = self.create_timer(0.1, self._control_loop)
        
        self.get_logger().info("BouncingNodeV2 initialized and ready")

    def _detections_callback(self, msg: Float32MultiArray) -> None:
        """
        Process YOLO detections from the consolidated topic.
        
        Format of the message:
        [num_detections, 
         det1_class_id, det1_center_x, det1_center_y, det1_width, det1_height,
         det2_class_id, det2_center_x, det2_center_y, det2_width, det2_height, 
         ...]
        
        Args:
            msg (Float32MultiArray): Consolidated detection data
        """
        self.last_detection_time = self.get_clock().now()
        
        # Parse the detection data
        data = msg.data
        if not data:
            self.detected_objects = []
            return
        
        num_detections = int(data[0])
        self.detected_objects = []
        
        # Process each detection (5 values per detection: class_id, cx, cy, width, height)
        for i in range(num_detections):
            idx = 1 + (i * 5)  # Skip the first value (num_detections)
            if idx + 4 < len(data):
                class_id = int(data[idx])
                cx = data[idx + 1]
                cy = data[idx + 2]
                width = data[idx + 3]
                height = data[idx + 4]
                
                self.detected_objects.append((class_id, cx, cy, width, height))
                
        if self.detected_objects:
            self.get_logger().debug(f"Received {num_detections} detections")
            
    def _find_target_in_detections(self) -> Optional[Tuple[float, float, float, float]]:
        """
        Find the target figure class in the current detection results.
        
        Returns:
            Optional[Tuple[float, float, float, float]]: (cx, cy, width, height) if found, None otherwise
        """
        if not self.detected_objects:
            return None
            
        # Look for target class in detected objects
        for obj in self.detected_objects:
            class_id, cx, cy, width, height = obj
            if class_id == self.figure_class:
                return (cx, cy, width, height)
        
        return None

    def _calculate_camera_offset_pixels(self, altitude: float) -> float:
        """
        Calculate the camera offset in pixels based on current altitude.
        
        Args:
            altitude (float): Current altitude in meters
            
        Returns:
            float: Offset in pixels
        """
        return ImageCalculus.calculate_offset_pixels(
            self.CAMERA_OFFSET,
            altitude,
            self.CAMERA_FOV,
            self.image_height
        )

    def _publish_pid_setpoints(self) -> None:
        """
        Publish the setpoints for PID controllers.
        X setpoint is the center of the image width.
        Y setpoint is the center of the image height plus camera offset.
        """
        # X setpoint (center of image width)
        x_setpoint = Float64()
        x_setpoint.data = self.image_width / 2.0
        
        # Y setpoint (center of image height + offset)
        y_offset = self._calculate_camera_offset_pixels(self.current_altitude)
        y_setpoint = Float64()
        y_setpoint.data = self.image_height / 2.0 + y_offset
        
        # Publish setpoints
        self.pid_x_setpoint_pub.publish(x_setpoint)
        self.pid_y_setpoint_pub.publish(y_setpoint)

    def _publish_pid_states(self, target_data: Tuple[float, float, float, float]) -> None:
        """
        Publish the current states (target center coordinates) for PID controllers.
        
        Args:
            target_data (Tuple[float, float, float, float]): (cx, cy, width, height) of the target
        """
        cx, cy, width, height = target_data
        
        x_state = Float64()
        x_state.data = cx
        
        y_state = Float64()
        y_state.data = cy
        
        self.pid_x_state_pub.publish(x_state)
        self.pid_y_state_pub.publish(y_state)

    def _is_centered(self) -> bool:
        """
        Check if the drone is centered over the target based on PID control efforts.
        
        Returns:
            bool: True if centered, False otherwise
        """
        return (abs(self.pid_x_effort) < self.CENTERING_THRESHOLD and 
                abs(self.pid_y_effort) < self.CENTERING_THRESHOLD)

    def _control_loop(self) -> None:
        """
        Main control loop for the mission.
        Called periodically to handle mission state and control the drone.
        """
        # Update current altitude
        self.current_altitude = self.drone._rel_alt.data
        
        # Skip if mission is complete
        if self.mission_complete:
            return
            
        # Look for target figure in detections
        target_data = self._find_target_in_detections()
        
        # If target is detected
        if target_data is not None:
            cx, cy, width, height = target_data
            self.figure_detected = True
            self.get_logger().info(f"Target detected at ({cx:.1f}, {cy:.1f}), size: {width:.1f}x{height:.1f}")
            
            # Publish current target position to PID controllers
            self._publish_pid_setpoints()
            self._publish_pid_states(target_data)
            
            # Apply control to center the drone
            # Scale PID efforts to velocity commands
            vel_x = -self.pid_y_effort  # Forward/backward (y-axis in image = x-axis in drone)
            vel_y = -self.pid_x_effort  # Left/right (x-axis in image = y-axis in drone)
            
            # Limit velocity to safe range
            vel_x = max(min(vel_x, self.MAX_VELOCITY), -self.MAX_VELOCITY)
            vel_y = max(min(vel_y, self.MAX_VELOCITY), -self.MAX_VELOCITY)
            
            # If drone is centered and not already in descent
            if self._is_centered() and not self.centering_complete:
                self.get_logger().info("Target centered! Starting descent.")
                self.centering_complete = True
            
            # Apply vertical velocity for descent if centered
            vel_z = 0.0
            if self.centering_complete:
                if self.current_altitude > self.FINAL_ALTITUDE:
                    vel_z = -self.DESCENT_SPEED
                else:
                    # At final altitude, initiate landing
                    self.get_logger().info(f"Reached target altitude of {self.FINAL_ALTITUDE}m. Initiating landing.")
                    self.land()
                    return
            
            # Apply velocity control
            self.drone.offboard_velocity(linear_x=vel_x, linear_y=vel_y, linear_z=vel_z)
            
        elif self.figure_detected:
            # If figure was detected before but not now, slow down and try to reacquire
            self.get_logger().warn("Lost target! Hovering to reacquire.")
            self.drone.offboard_velocity(linear_x=0.0, linear_y=0.0, linear_z=0.0)
            self.centering_complete = False
            
            # If we haven't seen target for too long, assume we lost it
            time_since_detection = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
            if time_since_detection > 3.0:  # 3 second timeout
                self.get_logger().error("Target lost for too long. Resuming search.")
                self.figure_detected = False
        else:
            # If no target detected and not in detection mode, continue searching
            self._continue_search()

    def _continue_search(self) -> None:
        """
        Continue searching for the target by visiting waypoints.
        """
        if not self.search_points:
            self.get_logger().error("No search points available.")
            return
            
        # Sort points by distance from current position
        sorted_points = sorted(
            self.search_points,
            key=lambda point: self.drone.gps_controller.haversine_distance(
                lat=point[0], 
                lon=point[1]
            )
        )
        
        # Go to the closest point
        next_point = sorted_points[0]
        self.get_logger().info(f"Moving to search point: {next_point}")
        
        # Remove the point we're visiting
        self.search_points.remove(next_point)
        
        # Move to the next point
        self.drone.offboard_gps_position(
            lat_setpoint=next_point[0],
            lon_setpoint=next_point[1],
            alt_setpoint=self.TAKEOFF_ALTITUDE,
            precision_radius=1.0  # 1-meter precision
        )
        
        # Wait for drone to reach the point
        sleep_time = 5.0  # 5 seconds should be enough to reach most points
        time.sleep(sleep_time)

    def run_mission(self) -> None:
        """
        Execute the full mission.
        - Take off
        - Search for target
        - Center over target
        - Land on target
        """
        self.get_logger().info("Starting mission")
        
        # Initialize and take off
        self.drone.arm_takeoff(self.TAKEOFF_ALTITUDE)
        time.sleep(8.0)  # Wait for takeoff to complete
        
        self.get_logger().info(f"Takeoff complete. Altitude: {self.current_altitude}m")
        
        # Start control loop - let the timer handle the mission
        try:
            while rclpy.ok() and not self.mission_complete:
                rclpy.spin_once(self, timeout_sec=0.1)
        except Exception as e:
            self.get_logger().error(f"Error during mission execution: {str(e)}")
            self.drone.land()  # Emergency landing
            
        self.get_logger().info("Mission complete")

    def land(self) -> None:
        """
        Land the drone and complete the mission.
        """
        self.get_logger().info("Landing on target")
        
        # Stop velocity control
        self.drone.offboard_velocity(0.0, 0.0, 0.0)
        time.sleep(1.0)
        
        # Execute landing
        self.drone.land()
        
        # Wait for landing to complete
        time.sleep(5.0)
        
        self.mission_complete = True
        self.get_logger().info("Landing complete. Mission successful.")

    def _pid_x_callback(self, msg: Float64) -> None:
        """
        Store the PID X-axis control effort.
        
        Args:
            msg (Float64): Control effort value
        """
        self.pid_x_effort = msg.data

    def _pid_y_callback(self, msg: Float64) -> None:
        """
        Store the PID Y-axis control effort.
        
        Args:
            msg (Float64): Control effort value
        """
        self.pid_y_effort = msg.data

    def _calculate_search_points(self) -> List[Tuple[float, float]]:
        """
        Calculate search waypoints within the specified GPS area.
        
        Returns:
            List[Tuple[float, float]]: List of (lat, lon) waypoints
        """
        # Calculate midpoints between corners
        mid_top = (
            (self.corner_top_left[0] + self.corner_top_right[0]) / 2,
            (self.corner_top_left[1] + self.corner_top_right[1]) / 2
        )
        
        mid_bottom = (
            (self.corner_bottom_left[0] + self.corner_bottom_right[0]) / 2,
            (self.corner_bottom_left[1] + self.corner_bottom_right[1]) / 2
        )
        
        mid_left = (
            (self.corner_top_left[0] + self.corner_bottom_left[0]) / 2,
            (self.corner_top_left[1] + self.corner_bottom_left[1]) / 2
        )
        
        mid_right = (
            (self.corner_top_right[0] + self.corner_bottom_right[0]) / 2,
            (self.corner_top_right[1] + self.corner_bottom_right[1]) / 2
        )
        
        # Calculate center
        center = (
            (self.corner_top_left[0] + self.corner_top_right[0] + 
             self.corner_bottom_left[0] + self.corner_bottom_right[0]) / 4,
            (self.corner_top_left[1] + self.corner_top_right[1] + 
             self.corner_bottom_left[1] + self.corner_bottom_right[1]) / 4
        )
        
        # Create grid of points
        search_points = [
            self.corner_top_left,
            self.corner_top_right,
            self.corner_bottom_left,
            self.corner_bottom_right,
            mid_top,
            mid_bottom,
            mid_left,
            mid_right,
            center
        ]
        
        return search_points

def main(args=None) -> None:
    """
    Main function to run the BouncingNodeV2.
    """
    rclpy.init(args=args)
    
    # Create and run the node
    node = BouncingNodeV2(
        figure="cross",
        p1_lat=-23.1979885, p1_lon=-45.909908,   # Top-left
        p2_lat=-23.1978449, p2_lon=-45.9098869,  # Top-right
        p3_lat=-23.1980094, p3_lon=-45.9098283,  # Bottom-left
        p4_lat=-23.1978557, p4_lon=-45.9098052   # Bottom-right
    )
    
    try:
        node.run_mission()
    except KeyboardInterrupt:
        node.get_logger().info("Mission aborted by user")
    except Exception as e:
        node.get_logger().error(f"Error during mission: {str(e)}")
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
