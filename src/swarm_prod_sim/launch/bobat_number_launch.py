import os
import rclpy

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController

from launch.actions import DeclareLaunchArgument, OpaqueFunction
import xacro




def launch_setup(context):
    # rclpy.init(args=[])
    # logger = rclpy.create_node("logger")

    namespace_config=LaunchConfiguration('bobat_namespace')
    namespace=namespace_config.perform(context)
    package_dir = get_package_share_directory('swarm_prod_sim')
    robot_description_path = os.path.join(package_dir, 'resource', 'webots_bobat.urdf')

    doc = xacro.process_file(package_dir + "/resource/" + "bobat.urdf.xacro", mappings={"robot_id": namespace + "/"})
    robot_desc = doc.toprettyxml(indent='  ')

    file_to_save_name = package_dir + "/resource/" + namespace + ".test.urdf"
    if os.path.exists(file_to_save_name):
        os.remove(file_to_save_name)
    file_handle = open(file_to_save_name, "w")
    doc.writexml(file_handle, indent='  ', newl='\n', encoding='utf-8')
    file_handle.close()



    # obstacle_avoider = Node(
    #     package='swarm_prod_sim',
    #     namespace=namespace,
    #     executable='obstacle_avoider',
    #     # name= '{bobat}_avoider'.format(bobat=namespace),
    # )

    simple_nav = Node(
        package='swarm_prod_sim',
        namespace=namespace,
        executable='simple_nav',
        parameters=[{"bobat_id": namespace,}]
    )

    bobat_broadcaster = Node(
        package='swarm_prod_sim',
        namespace=namespace,
        executable='bobat_broadcaster',
        # name= '{bobat}_broadcaster'.format(bobat=namespace),
        parameters=[{"bobat_id": namespace,
                     "robot_description": robot_desc,}],
    )

    # logger.get_logger().info("stuff:")
    # logger.get_logger().info(namespace)
    # logger.get_logger().info(bobat_broadcaster.name)

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        # name= '{bobat}_state_publisher'.format(bobat=namespace),
        parameters=[{"robot_description": robot_desc}],
    )

    my_robot_driver = WebotsController(
        robot_name=namespace,
        parameters=[
            {'robot_description': robot_description_path},
        ],
        remappings=[('/cmd_vel', '/' + namespace + '/cmd_vel_smoothed'),
                    ('/gps', '/' + namespace + '/gps'),
                    ('/compass/bearing', '/' + namespace + '/compass/bearing'),
                    ('/compass/north_vector', '/' + namespace + '/compass/north_vector'),
                    ('/gps/speed', '/' + namespace + '/gps/speed'),
                    ('/gps/speed_vector', '/' + namespace + '/gps/speed_vector'),
                    ('/left_sensor', '/' + namespace + '/left_sensor'),
                    ('/right_sensor', '/' + namespace + '/right_sensor')],
        namespace=namespace

    )


    return [
            # obstacle_avoider,
            simple_nav,
            bobat_broadcaster,
            state_publisher,
            my_robot_driver]



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

