import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.service import SrvTypeRequest

from mavros_msgs.srv import (
    CommandTOL,
)

class Land_Command(Node):
    def __init__(
            self
        ) -> None:

        super().__init__("land_command_node")

        self._land_srv = self._create_client(CommandTOL, "/mavros/cmd/land")
    def _create_client(self, srv_type, service_name: str):
        """
        Helper function to create a ROS2 service client.

        :param srv_type: ROS2 service type.
        :param service_name (str): ROS2 service name.
        """
        client = self.create_client(srv_type, service_name)
        self._clients.append(client)
        return client

    def _call_service(
        self,
        service: Client,
        request: SrvTypeRequest,
        success_message: str,
        failure_message: str,
        sync: bool = False,
    ):
        """
        Auxiliar function to call services and print result.

        :param service (Client): Service client
        :param request (Request): Service request
        :param success_message (str): Message to print if success
        :param failure_message (str): Message to print if failure
        :param sync: If True, call the service synchronously, otherwise asynchronously.
            Synchoronous call will block the code until the service is done
        """

        def _wait_for_service():
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    f"Service {service.srv_name} not available, waiting again..."
                )

        def _print_result(result):
            if result is not None:
                self.get_logger().info(f"\033[32;1;4m{success_message}\033[0m")
            else:
                self.get_logger().error(f"\033[31;1;4m{failure_message}\033[0m")

        def _handle_future(future):
            try:
                result = future.result()
            except Exception as e:
                self.get_logger().error(
                    f"Service call failed {service.srv_name}: {str(e)}"
                )
                result = None
            finally:
                _print_result(result)

        _wait_for_service()
        self.get_logger().info(
            f"-- Calling service {service.srv_name} | Sync: {sync}"
        )

        if sync:
            result = service.call(request)
            _print_result(result)
        else:
            future = service.call_async(request)
            future.add_done_callback(_handle_future)

    def land(self):
        """
        Send command to land the drone.
        """
        req = CommandTOL.Request()
        req.altitude = 0.0
        self._call_service(self._land_srv, req, "-- Landed", "-- Land failed")

def main():
    rclpy.init()
    lc = Land_Command()
    lc.land()

if __name__ == "__main__":
    main()

    
