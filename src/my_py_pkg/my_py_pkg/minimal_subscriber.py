import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from example_interfaces.msg import String


class MinimalSubscriber(Node):
    """Subscriber thats prints message into the console"""

    def __init__(self):
        """Constructor"""

        super().__init__("minimal_subscriber")

        self._subscriber = self.create_subscription(String, 'my_topic', self._listerner_callback, 10)

    def _listerner_callback(self, msg):
        """Prints the message to the console"""
        self.get_logger().info(f"Received message: {msg.data}")
        

def main(args=None):
    
    try:
        rclpy.init()
        node = MinimalSubscriber()
        rclpy.spin(node)

    except(KeyboardInterrupt, ExternalShutdownException):
        ...
        
    finally:
        if node is not None:
            node.destroy_node()
            
        if rclpy.ok():
            rclpy.shutdown()
            

if __name__ == "__main__":
    main()