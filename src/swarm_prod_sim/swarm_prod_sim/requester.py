import rclpy
from rclpy.impl.logging_severity import LoggingSeverity
import sys

from rclpy.node import Node as RealNode
from launch_ros.actions import Node as LaunchRosNode
from std_msgs.msg import Int32
from webots_ros2_msgs.srv import SpawnNodeFromString
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess

from webots_ros2_driver.webots_controller import WebotsController



class Requester(RealNode):
    def __init__(self):
        super().__init__('requester')
        self.get_logger().set_level(LoggingSeverity.DEBUG)


        self.something = "bobat { name \"bobat-1\"}"

        # self.launcher = LaunchService(argv=sys.argv[1:])
        self.counter = 0
        package_dir = get_package_share_directory('swarm_prod_sim')
        self.robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')


        self.client = self.create_client(SpawnNodeFromString, 'Ros2Supervisor/spawn_node_from_string')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = SpawnNodeFromString.Request()
        self.create_subscription(Int32, 'command_string', self.__command_string_callback, 1)

    def generate_a_launch_description(self, ns=""):
        namespace = ns
        my_robot_driver = WebotsController(
            # name='my_robot_driver',
            robot_name=namespace,
            parameters=[
                {'robot_description': self.robot_description_path},
            ],
            remappings=[('/cmd_vel', '/' + namespace + '/cmd_vel'),
                        ('/left_sensor', '/' + namespace + '/left_sensor'),
                        ('/right_sensor', '/' + namespace + '/right_sensor')],
            namespace=namespace
        )

        obstacle_avoider = LaunchRosNode(
            package='swarm_prod_sim',
            namespace=namespace,
            executable='obstacle_avoider',
        )


        return LaunchDescription([my_robot_driver,
                                  obstacle_avoider,])

    def send_request(self, command):
        self.request.data = command
        self.get_logger().info('Sending request... ' + self.request.data)
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def __command_string_callback(self, msg):
        if msg.data == 1:
            self.counter+=1
            namespace = "bobat-" + str(self.counter)
            self.get_logger().info("callback initiated")


            # self.launcher.include_launch_description(self.generate_a_launch_description(namespace))
            # self.launcher.run()

            # self.send_request(self.something)
            # self.send_request(self.something.replace("VAR_NAME", namespace))


            # self.launcher.include_launch_description(self.generate_a_launch_description(namespace))
            # self.launcher.run()

            # os.system("ros2 launch swarm_prod_sim launch_worldly_bobat.py")


        return

def main(args=None):
    rclpy.init(args=args)
    MeNode = Requester()
    rclpy.spin(MeNode)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    MeNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()