import os

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController
from launch.actions import LogInfo



from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context):
    namespace_config=LaunchConfiguration('bobat_namespace')
    namespace=namespace_config.perform(context)
    package_dir = get_package_share_directory('my_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

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

    return [obstacle_avoider, my_robot_driver]



def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'bobat_namespace',
            default_value="Worldly",
            description='name for a bobat, typically the same followed by _#',
        )
    )



    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

