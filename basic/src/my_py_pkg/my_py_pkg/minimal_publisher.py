import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from example_interfaces.msg import String


class MinimalPublisher(Node):
    """Publisher example that periodically sends out a string"""

    def __init__(self):
        """Constructor"""

        # call the Node class constructor with the node name
        super().__init__("minimal_publisher")

        # create a publisher object
        self._publisher = self.create_publisher(String, 'my_topic', 10)
        
        # Periodically call method
        self._timer = self.create_timer(0.5, self._timer_callback)
        
        # Counter for message send
        self._counter = 0
        
    def _timer_callback(self):
        """Publishes a simple message to topic"""

        # Fill out string message
        msg = String()
        msg.data = f"Hello {self._counter}"

        # publish message to topic
        self._publisher.publish(msg)
        self.get_logger().info(f"Publisher: {msg.data}")

        # Increment counter
        self._counter += 1
            

def main(args=None):
    """Main entrypoint"""

    # Initialize and run node
    try:
        rclpy.init()
        node = MinimalPublisher()
        rclpy.spin(node)

    # Catch ctrl+c or shutdown request
    except(KeyboardInterrupt, ExternalShutdownException):
        ...
        
    finally:
        if node is not None:
            node.destroy_node()
            
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main() 