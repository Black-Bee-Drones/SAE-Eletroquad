import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import String, Float32MultiArray
from mirela_sdk.control.mavros.mavros_api import MavDrone
import threading
import time

class DroneCommand:
    """ Encapsulates the drone command functionality.
        It is used to send commands to the drone and receive status updates.
    """
    def __init__(self, drone, status_pub, logger):
        self.drone = drone
        self.status_pub = status_pub
        self.logger = logger
        self.command_thread = None


    def execute_async(self, command_func, completion_status, *args, **kwargs):
        """ Executes a command asynchronously in a separate thread.
            Args:
                command_func: The function to execute.
                completion_status: The status to publish upon completion.
                *args: Positional arguments for the command function.
                **kwargs: Keyword arguments for the command function.
        """
        if self.command_thread and self.command_thread.is_alive():
            self.logger.warn("Command already in progress. Ignoring new command.")

        
        def task():
            try:
                command_func(*args, **kwargs) # execute the command with the provided arguments
                msg = String()
                msg.data = completion_status
                self.status_pub.publish(msg)
                self.logger.info(f"Command '{command_func.__name__}' executed sucessfully.")
            
            except Exception as e:
                self.logger.error(f"Error executing command: {e}")
            
        # create and start the thread for executing the command
        self._command_thread = threading.Thread(target=task)
        self._command_thread.daemon = True # ends when the main thread ends
        self._command_thread.start() # start the thread
