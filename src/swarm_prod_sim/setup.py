from setuptools import setup

package_name = 'swarm_prod_sim'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/launch_world_and_worldly.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/launch_worldly_bobat.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/any_bobat_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/any_bob_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/test_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/launch_arena_five.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/launch_epuck_overlauncher.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/bobat_number_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/launch_5_bobat.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/place_4_bob.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/empty_factory.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/any_bobat_everything_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/many_bots.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/emptyish_arena.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/factory.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/webots_bobat.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/bobat.urdf.xacro']))
data_files.append(('share/' + package_name + '/resource', ['resource/Webots_robot_string.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/bobat.proto']))
data_files.append(('share/' + package_name + '/resource', ['resource/bob.proto']))
data_files.append(('share/' + package_name + '/resource', ['resource/bobat.rviz']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = swarm_prod_sim.my_robot_driver:main',
            'requester = swarm_prod_sim.requester:main',
            'obstacle_avoider = swarm_prod_sim.obstacle_avoider:main',
            'bobat_broadcaster = swarm_prod_sim.bobat_broadcaster:main',
            'simple_nav = swarm_prod_sim.simple_nav:main',
        ],
    },
)
