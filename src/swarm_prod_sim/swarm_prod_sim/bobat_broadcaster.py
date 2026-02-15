import rclpy
from rclpy.node import Node
import math

import tf2_ros
from geometry_msgs.msg import TransformStamped, PointStamped, Quaternion, Vector3
from webots_ros2_msgs.msg import FloatStamped


class BobatBroadcaster(Node):
    def __init__(self):
        super().__init__('bobat_broadcaster')

        self.declare_parameter('bobat_id', "bobat_1")
        self.bobat_id: str = self.get_parameter('bobat_id').get_parameter_value().string_value

        self.declare_parameter('world_frame_id', "world")
        self.world_frame_id: str = self.get_parameter('world_frame_id').get_parameter_value().string_value

        self.declare_parameter("robot_description", "")
        self.robot_description: str = self.get_parameter('robot_description').get_parameter_value().string_value

        # self.desc_pub = self.create_publisher(String, self.get_namespace() + '/robot_description', 10)

        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        # self.get_logger().info('bobat_broadcaster is about to for loop')
        for key, value in {
            "base_link": (self.bobat_id, [0.0, 0.0, 0.0]),
            "left_wheel": ("base_link", [0.0, 0.045, 0.025]),
            "right_wheel": ("base_link", [0.0, -0.045, 0.025]), }.items():
                # self.get_logger().info(f"doing {key} with {value}")
                static_transform_stamped = TransformStamped()
                if value[0] != self.bobat_id:
                    static_transform_stamped.header.frame_id = self.bobat_id + "/" + value[0]
                else:
                    static_transform_stamped.header.frame_id = value[0]
                static_transform_stamped.child_frame_id = self.bobat_id + "/" + key
                translation = Vector3()
                translation.x = value[1][0]
                translation.y = value[1][1]
                translation.z = value[1][2]
                static_transform_stamped.transform.translation = translation

                # self.get_logger().info('static_transform_stamped.transform.translation')
                # self.get_logger().info(str(static_transform_stamped.transform.translation))
                self.tf_static_broadcaster.sendTransform(static_transform_stamped)
                pass

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer_period: float = 0.1
        self.timer_elapsed_time: float = 0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.create_subscription(FloatStamped, "compass/bearing", self.__compass_callback, 1)
        self.create_subscription(PointStamped, "gps", self.__gps_callback, 1)

        self.transform_stamped = TransformStamped()

    def timer_callback(self):
        tfs = TransformStamped()

        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id = self.world_frame_id
        tfs.child_frame_id = self.bobat_id

        tfs.transform.translation = self.transform_stamped.transform.translation
        tfs.transform.rotation = self.transform_stamped.transform.rotation

        self.timer_elapsed_time += self.timer_period

        # FOR DEBUG
        # self.get_logger().info("tfs is")
        # self.get_logger().info(str(tfs))
        # # FOR MORE DEBUG
        # self.get_logger().info("tfs.header is")
        # self.get_logger().info(str(tfs.header))
        # self.get_logger().info("tfs.transform is")
        # self.get_logger().info(str(tfs.transform))

        self.tf_broadcaster.sendTransform(tfs)

        # msg = String()
        # msg.data = self.robot_description
        # self.desc_pub.publish(msg)

    @staticmethod
    def quat_from_z_rotation(theta: float) -> Quaternion:
        """
        Create a quaternion from a rotation about the z-axis.

        Parameters:
            theta (float): rotation angle in degrees

        """
        half_theta = -theta / 2.0
        return Quaternion(
            w= math.cos(math.radians(half_theta)),  # w
            x= 0.0,  # x
            y= 0.0,  # y
            z= math.sin(math.radians(half_theta))  # z
        )


    def __compass_callback(self, msg):
        # self.pose.transform.rotation = self.quat_from_z_rotation(msg.data)
        quat = self.quat_from_z_rotation(msg.data)
        self.transform_stamped.transform.rotation.w = quat.w
        self.transform_stamped.transform.rotation.x = quat.x
        self.transform_stamped.transform.rotation.y = quat.y
        self.transform_stamped.transform.rotation.z = quat.z
        # self.get_logger().info("Compass callback called")

    def __gps_callback(self, msg):
        # self.pose.transform.translation = msg.point
        self.transform_stamped.transform.translation.x = msg.point.x
        self.transform_stamped.transform.translation.y = msg.point.y
        self.transform_stamped.transform.translation.z = msg.point.z
        # self.get_logger().info("GPS callback called")

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)
        broad_caster = BobatBroadcaster()

        # broad_caster.get_logger().info("BobatBroadcaster started")
        # broad_caster.get_logger().info(broad_caster.get_name())
        # broad_caster.get_logger().info(broad_caster.bobat_id)

        rclpy.spin(broad_caster)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
