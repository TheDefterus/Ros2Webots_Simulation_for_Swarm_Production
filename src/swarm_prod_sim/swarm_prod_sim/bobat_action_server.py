import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

something = ""

class BobatActionServer(Node):

    def __init__(self):
        super().__init__('bobat_action_server')
        self._action_server = ActionServer(
            self,
            something,
            "something",
            self.execute_callback
        )

    def execute_callback(self, action_handle):
        self.get_logger().info("Executing action...")
        result = something.result()
        return result

def main(args=None):
    rclpy.init(args=args)
    bobat_action_server = BobatActionServer()
    rclpy.spin(bobat_action_server)

if __name__ == '__main__':
    main()