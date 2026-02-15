from logging import DEBUG

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped

import tf2_ros
import math
# from swarm_prod_sim_interfaces.action import  MoveStraight2D

MAX_RANGE = 0.15

class SimpleNav(Node):
    def __init__(self):
        super().__init__('simple_nav')

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.create_subscription(Range, 'left_sensor', self.__left_sensor_callback, 1)
        self.left_eye = 0.0
        self.create_subscription(Range, 'right_sensor', self.__right_sensor_callback, 1)
        self.right_eye = 0.0

        self.transform_listener_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.transform_listener_buffer, self)
        self.declare_parameter('bobat_id', "bobat_1")
        self.bobat_id: str = self.get_parameter('bobat_id').get_parameter_value().string_value
        self.parent_id = 'world'

        self.timer_period: float = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.pose = TransformStamped()
        self.goal = ""
        self.create_subscription(PoseStamped, "goal_pose", self.__goal_callback, 1)

    @staticmethod
    def goal_error(pose, goal):
        sx = pose.transform.translation.x
        sy = pose.transform.translation.y
        gx = goal.pose.position.x
        gy = goal.pose.position.y
        return math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2)

    @staticmethod
    def goal_heading_error(pose, goal):
        dx = goal.pose.position.x - pose.transform.translation.x
        dy = goal.pose.position.y - pose.transform.translation.y
        target_heading = math.atan2(dy, dx)
        q = pose.transform.rotation
        current_heading = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        error = current_heading - target_heading
        while error > math.pi: error -= 2.0 * math.pi
        while error < -math.pi: error += 2.0 * math.pi
        return error

    def timer_callback(self):
        msg = Twist()
        try:
            tfs = self.transform_listener_buffer.lookup_transform(
                self.parent_id,
                self.bobat_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=5),
            )
            self.pose = tfs
            if self.goal == "":
                self.goal = PoseStamped()
                self.goal.pose.position.x = self.pose.transform.translation.x
                self.goal.pose.position.y = self.pose.transform.translation.y
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'Could not get transform from `{self.parent_id}` to `{self.bobat_id}`: {e}')
        finally:
            if self.goal == "":
                return
            goal_error = self.goal_error(self.pose, self.goal)
            heading_error = 0.0
            if goal_error>0.045:
                msg.linear.x = 0.1
                heading_error = self.goal_heading_error(self.pose, self.goal)
                if abs(heading_error) > 0.1:
                    msg.angular.z = -1.0 if heading_error > 0 else 1.0
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            self.__publisher.publish(msg)

        # self.get_logger().info(f'to Goal: {goal_error}')
        # self.get_logger().info(f'to GoalHeading: {heading_error}')




    def __goal_callback(self, goal):
        self.goal = goal
        # self.get_logger().info(f'Goal: {self.goal}')

    def __left_sensor_callback(self, msg):
        self.left_eye = msg.range

    def __right_sensor_callback(self, msg):
        self.right_eye = msg.range

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)
        simple_nav = SimpleNav()
        rclpy.spin(simple_nav)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()

