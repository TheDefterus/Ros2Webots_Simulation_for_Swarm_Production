from tkinter.font import names
from tokenize import String

import rclpy
from rclpy.impl.logging_severity import LoggingSeverity

from rclpy.node import Node as RealNode
from launch_ros.actions import Node as LaunchRosNode
from std_msgs.msg import Int32
import sys
from webots_ros2_msgs.srv import SpawnNodeFromString
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch import LaunchService

from webots_ros2_driver.webots_controller import WebotsController



class Requester(RealNode):
    def __init__(self):
        super().__init__('requester')
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        self.something = open(os.path.join(get_package_share_directory('my_package'), 'resource', 'Webots_robot_string.wbt')).read()[2:]
        # self.get_logger().info(self.something)
        self.launcher = LaunchService(noninteractive=False)
        self.counter = 0
        package_dir = get_package_share_directory('my_package')
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
            package='my_package',
            namespace=namespace,
            executable='obstacle_avoider',
        )


        return LaunchDescription([my_robot_driver,
                                  obstacle_avoider,])

    def send_request(self, command):
        self.request.data = command
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def __command_string_callback(self, msg):
        if msg.data == 1:
            self.counter+=1
            namespace = "bobat-" + str(self.counter)


            # self.launcher.include_launch_description(self.generate_a_launch_description(namespace))
            # self.launcher.run()

            self.send_request(self.something.replace("HopefullyVariable", namespace))


            self.launcher.include_launch_description(self.generate_a_launch_description(namespace))
            self.launcher.run()
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