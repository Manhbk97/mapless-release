import math

from geometry_msgs.msg import Twist
from rclpy.clock import Clock
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist, Point, Pose,  Vector3, Quaternion
import numpy as np 
from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'map', 'base_link').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create odom publisher
        self.odom_pub = self.create_publisher(Odometry, 'baselink2map', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations



        try:
            t= self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base link to odom: {ex}')
            return
        # print( t.transform.rotation)
        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        time_stamp = Clock().now()
        odom.header.stamp  = time_stamp.to_msg()  
        # odom.header.stamp = rclpy.time.Time(0)
        odom.header.frame_id = "map"
        # print("trans: "+ str(trans))
        # print("rot:" + str(rot))
        # set the position
        odom.pose.pose.position.x = t.transform.translation.x
        odom.pose.pose.position.y = t.transform.translation.y
        odom.pose.pose.position.z = t.transform.translation.z

        odom.pose.pose.orientation.x = t.transform.rotation.x
        odom.pose.pose.orientation.y = t.transform.rotation.y
        odom.pose.pose.orientation.z = t.transform.rotation.z
        odom.pose.pose.orientation.w = t.transform.rotation.w


        odom.child_frame_id = "base_link"
        # odom.twist.twist = rot
        (qx, qy, qz, qw) = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.z)    
        # print("position:",posx, posy, posz )    
        # print("orientation",qx, qy, qz, qw )
        orientation_list = [qx, qy, qz, qw]
        # print("orientation", orientation_list)
        # orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = self.euler_from_quaternion(orientation_list)
        # publish the message
        self.odom_pub.publish(odom)
        print("Position 2d: ")
        print("x:", odom.pose.pose.position.x)
        print("y", odom.pose.pose.position.y)
        # print(odom.pose.pose.position.y)
        print("yaw: ", yaw)


    def euler_from_quaternion(self,quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()