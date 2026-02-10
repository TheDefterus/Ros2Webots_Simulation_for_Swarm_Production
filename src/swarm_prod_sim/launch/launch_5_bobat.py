from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def launch_setup(context):
    package_dir = get_package_share_directory("my_package")
    number = 5
    launches = []
    launches.append(
        IncludeLaunchDescription(
            package_dir + "/launch/" + "test_launch.py",
        )
    )
    launches.append(
        IncludeLaunchDescription(
            package_dir + "/launch/" + "any_bobat_launch.py",
            launch_arguments={
                'number': str(number),
            }.items(),
        )
    )
    return launches

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])