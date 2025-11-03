import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


class MinimalServer(Node):
    """Server Example that adds two integers and respond with result"""

    def __init__(self):
        """Constructor"""

        # Call the Node class constructor with the Node name
        super().__init__("minimal_server")

        # Create server object
        self._srv = self.create_service(AddTwoInts, 'add_ints', self._server_callback)
        
    def _server_callback(self, req, resp):
        """Response with the sum of ints"""
        resp.sum = req.a + req.b
        self.get_logger().info(f"Received request: a={req.a} and b={req.b}")
        
        return resp


def main(args=None):
    """Main entrypoint"""

    # Initialize and run Node
    try:
        rclpy.init()
        node = MinimalServer()
        rclpy.spin(node)

    # Catch ctrl+c and shutdown request
    except(KeyboardInterrupt, ExternalShutdownException):
        ...
        
    finally:
        if node is not None:
            node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()
            
if __name__ == "__main__":
    main()
