import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from mirela_sdk.image_processing import line
from mirela_sdk.control.mavros.mavros_api import MavDrone
from time import sleep
from enum import Enum, auto

class DroneState(Enum):
    IDLE = auto()
    TAKING_OFF = auto()
    FOLLOW_BLUE_LINE = auto()
    FIND_RED_LINE = auto()
    DELIVER_HOOK() = auto()
    RETURN_TO_LINE() = auto()
    LAND() = auto()
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
    
        # object drone of the class MavDrone    
        self.drone = MavDrone(node=self.node, mavros=False)
        self.node.get_logger().info("Unified Drone Controller has been initialized")

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
    
    async def execute_mission(self):
        try:
            # Activate if lifecycle node
            if self.is_lifecycle and not await self.ensure_active():
                self.node.get_logger().error("Failed to activate the drone.")
                return
            
            await self.takeoff_sequence()
            await self.follow_blue_line()
            await self.find_red_line()
            await self.deliver_hook()
            await self.return_to_line()
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
        await self.drone.arm_takeoff(2.5)
        await sleep(4)

    async def follow_blue_line(self):
        self.current_state = DroneState.FOLLOW_BLUE_LINE
        self.node.get_logger().info("Follow line initialized")

    async def find_red_line(self):
        self.current_state = DroneState.FIND_RED_LINE
        self.node.get_logger().info("Finding red line...")
        await sleep(2)
        
    async def deliver_hook(self):
        self.current_state = DroneState.DELIVER_HOOK
        self.node.get_logger().info("Delivering hook...")
        await sleep(2)

    async def return_to_line(self):
        self.current_state = DroneState.RETURN_TO_LINE
        self.node.get_logger().info("Returning to line...")
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

    


