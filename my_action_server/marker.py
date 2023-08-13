#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist ,Point, Pose,  Vector3, Quaternion
from std_msgs.msg import String
import numpy as np
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.clock import Clock
from math import pi, sqrt, pow, exp
from rclpy.qos import ReliabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TwistStamped, Pose, Point, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA, String


class TrajectoryInteractiveMarkers(Node):

    def __init__(self):
        super().__init__('maker_odom')
        self.count = 0 
        self._marker_publisher = self.create_publisher(Marker,'odom_marker',10)
        self.sub_odom = self.create_subscription(Odometry,'/baselink2map', self.event_in_cb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.sub_odom = self.create_subscription(Odometry,'/odom', self.event_in_cb, QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))

    def event_in_cb(self,msg):
        self.odometry = msg
        self.a = list()
        self.a.append(self.odometry.pose.pose.position.x)
        self.a.append(self.odometry.pose.pose.position.y)
        self.a.append(self.odometry.pose.pose.position.z)
        print(self.a)
        self.show_text_in_rviz()

    def show_text_in_rviz(self):
        # self.marker = Marker()
        # self.marker = Marker(
        #     type=Marker.CYLINDER,
        #     id=0,
        #     # lifetime=rospy.Duration(500),
        #     pose=Pose(Point(self.a[0]/10**5,self.a[1]/10**5,self.a[2]/10**5), Quaternion(0, 0, 0, 1)),
        #     scale=Vector3(0.02, 0.02, 0.02),
        #     header=Header(frame_id='base_link'),
        #     color=ColorRGBA(0.0, 2.0, 0.0, 0.8))
        
        marker_ = Marker()
        # marker_.header.frame_id = "map"
        marker_.header.frame_id = "base_link"
        # marker_.header.child_frame_id = "base_link"
        # # # marker_.header.stamp = rospy.Time.now()
        time_stamp = Clock().now()
        marker_.header.stamp   = time_stamp.to_msg()  
        marker_.type = marker_.CYLINDER
        # marker_.action = marker_.ADD


        marker_.pose.position.x = self.a[0]/(10**1)
        marker_.pose.position.y = self.a[1]/(10**1)
        marker_.pose.position.z = self.a[2]/(10**1)
        marker_.pose.orientation.x = 0.0
        marker_.pose.orientation.y = -0.8
        marker_.pose.orientation.z = 0.0
        marker_.pose.orientation.w = 1.0


        # marker_.lifetime = rospy.Duration.from_sec(lifetime_)
        marker_.scale.x = 0.05
        marker_.scale.y = 0.05
        marker_.scale.z = 0.02
        marker_.color.a = 0.8
        red_, green_, blue_ = 0.0,0.0,0.0
        marker_.color.r = 10.0
        marker_.color.g = 2.0
        marker_.color.b = 0.0


        self.count+=1
        marker_.id = self.count



        self._marker_publisher.publish(marker_)
        #rospy.loginfo('msg published')


def main(args=None):
    rclpy.init(args=args)
    # a = MinimalSubscriber()
    minimal_subscriber = TrajectoryInteractiveMarkers()
    # print("1")
    # minimal_subscriber.get_logger().info("Hello friend!")
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()