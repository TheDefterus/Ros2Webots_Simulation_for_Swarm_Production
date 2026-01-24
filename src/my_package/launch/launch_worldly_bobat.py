import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('my_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    launch_dir = os.path.join(package_dir, 'launch')
    namespace="Worldly"

    obstacle_avoider = Node(
        package='my_package',
        namespace=namespace,
        executable='obstacle_avoider',
    )

    my_robot_driver = WebotsController(
        robot_name=namespace,
        parameters=[
            {'robot_description': robot_description_path},
        ],
        remappings=[('/cmd_vel', '/' + namespace + '/cmd_vel'),
                    ('/left_sensor', '/' + namespace + '/left_sensor'),
                    ('/right_sensor', '/' + namespace + '/right_sensor')],
        namespace=namespace

    )

    return LaunchDescription([
        my_robot_driver,
        obstacle_avoider,
    ])

