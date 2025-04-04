import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from mirela_sdk.image_processing import line
from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.image_processing.line import LineDetector
from mirela_sdk.image_processing.camera import ImageHandler
from time import sleep
from enum import Enum, auto

class DroneState(Enum):
    IDLE = auto()
    TAKING_OFF = auto()
    FIND_RED_LINE = auto()
    DELIVER_HOOK = auto()
    RETURN_TO_HOME = auto()
    LAND = auto()
    COMPLETE = auto()
    

class MissionManager:
    def __init__(self, node, is_lifecycle=False, lifecycle_node_name=None):
        """
        @param node: The ROS2 node instance.
        @param is_lifecycle: Boolean indicating if the drone is a lifecycle node.
        @param lifecycle_node_name: The name of the lifecycle node (if applicable).
        """

        self.node = node
        self.is_lifecycle = is_lifecycle
        self.lifecycle_node_name = lifecycle_node_name
        
        if is_lifecycle:
            # If the drone is a lifecycle node, initialize the lifecycle client to change its states
            self.control_client = self.node.create_client(
                ChangeState,
                f"/{self.lifecycle_node_name}/change_state",
            )
    
        self.drone = MavDrone(node=self.node, mavros=False)
        self.find_line = line.LineDetector(color="red")
        self.image_handler = ImageHandler(node=self.node, 
                                          image_source="webcam",
                                          image_processing_callback=self._process_image,
                                          show_result=None,
                                          cap=0)
        self.node.get_logger().info("Unified Drone Controller has been initialized")

        self.drone_status_subscriber = self.node.create_subscription(
            DroneState,
            '/drone/state',
            self.drone_status_callback,
            10
        )

    def _process_image(self, image):
        """ Callback function to process the image. """
        if self.current_state == DroneState.FIND_RED_LINE:
            self.find_line.detect_line(frame=image, region=(400, 400))

    async def _publish_mission_status(self):
        """
        Publish the current mission status.
        """
        msg = DroneState()
        msg.state = self.current_state.name
        self.node.get_logger().info(f"Publishing mission status: {msg.status}")
        self.mission_status_publisher.publish(msg)

    async def drone_status_callback(self, msg):
        """
        Callback for drone status updates.
        """
        self.current_drone_status = msg
        self.node.get_logger().info(f"Drone status updated: {msg}")
        if msg.emergency:
            await self.emergency_land()

    async def ensure_active(self):
        """
        Ensure that the drone is in the active state.
        """
        if not self.is_lifecycle:
            return True
        
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        await self.change_state(transition)
        sleep(2) # we need to allow time for the state change, because it's not instantaneous

        transition.id = Transition.TRANSITION_ACTIVATE
        return await self.change_state(transition)
    
    async def change_state(self, transition):
        """
        Change the state of the lifecycle node.
        """
        if not self.control_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().error("Lifecycle service not available.")
            return False


        request = ChangeState.Request()
        request.transition = transition
        future = self.control_client.call_async(request)
        await future
        return future.result().sucess
    
    async def execute_mission(self):
        try:
            # Activate if lifecycle node
            if self.is_lifecycle and not await self.ensure_active():
                self.node.get_logger().error("Failed to activate the drone.")
                return
            
            await self.takeoff_sequence()
            await self.find_red_line()
            await self.deliver_hook()
            await self.return_to_home()
            await self.land_sequence()

            self.current_state = DroneState.COMPLETE
            self.node.get_logger().info("Mission completed successfully.")

        except Exception as e:
            self.node.get_logger().error(f"Mission failed: {e}")
            self.current_state = DroneState.ERROR
            await self.emergency_land()
            
    async def takeoff_sequence(self):
        self.current_state = DroneState.TAKING_OFF
        self.node.get_logger().info("Taking off...")
        await self.drone.arm_takeoff(4.0)
        await sleep(4)

    async def find_red_line(self):
        self.current_state = DroneState.FIND_RED_LINE
        self.node.get_logger().info("Searching for the red line...")
        self.image_handler.run()
        self.find_line.detect_line(frame=self.image_handler, region=(400, 400))
        await sleep(2)
        
    async def deliver_hook(self):
        self.current_state = DroneState.DELIVER_HOOK
        self.node.get_logger().info("Delivering hook...")
        await sleep(2)

    async def return_to_home(self):
        self.current_state = DroneState.RETURN_TO_HOME
        self.node.get_logger().info("Returning to home...")
        await self.drone.rtl(rtl_alt=4.0)
        await sleep(2)

    async def land_sequence(self):
        self.current_state = DroneState.LAND
        self.node.get_logger().info("Landing...")
        await self.drone.land()
        await sleep(2)
    
    async def emergency_land(self):
        self.node.get_logger().info("Emergency landing...")
        await self.drone.land()
        await sleep(2)
        self.current_state = DroneState.IDLE

    


