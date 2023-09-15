#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped, QuaternionStamped, Pose, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
import tf
import math

class OdometryPublisher:

    def __init__(self):

        self.last_odom_time = rospy.Time.now()
        self.x = 0
        self.y = 0
        self.v_x = 0
        self.v_y = 0
        self.delta_theta = 0
        self.delta_t = 0.001 # small value in order t
        self.init_theta = None
        self.twist = TwistStamped()
        self.quaternion = QuaternionStamped()

        rospy.init_node('odometry_publisher')
        rospy.Subscriber('filter/twist', TwistStamped, self.twist_callback)
        rospy.Subscriber('filter/quaternion', QuaternionStamped, self.quaternion_callback)
        self.odom_pub = rospy.Publisher('odometry', Odometry, queue_size=10)

        # create tf broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()


    def twist_callback(self, data):
        self.twist = data
        self.delta_t = (data.header.stamp - self.last_odom_time).to_sec()
        self.last_odom_time = data.header.stamp
        self.v_x = data.twist.linear.x
        self.v_y = data.twist.linear.y
        
    def calculate_position(self):
        vx = self.v_x * math.cos(self.delta_theta) - self.v_y * math.sin(self.delta_theta)
        vy = self.v_y * math.cos(self.delta_theta) + self.v_x * math.sin(self.delta_theta)
        self.x += vx * self.delta_t
        self.y += vy * self.delta_t

    def quaternion_callback(self, data):
        self.quaternion = data #from callback
        euler_angles = tf.transformations.euler_from_quaternion(self.quaternion.quaternion) #angle to radians
        theta = euler_angles[2] #yaw
        if self.init_theta is None: #from init_node
            self.init_theta = theta
        else:
            self.delta_theta = theta - self.init_theta


    def publish_odometry(self):

        current_time = rospy.Time.now()

        # create pose message
        pose = PoseWithCovariance()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.quaternion

        # create twist message
        twist = TwistWithCovariance()
        twist.twist = self.twist

        # create odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose = pose
        odom.twist = twist

        # publish odometry message
        self.odom_pub.publish(odom)

        translation = (self.x, self.y, 0)

    # Set the rotation component of the transform as a quaternion
        quaternion = (self.quaternion.quaternion.x, self.quaternion.quaternion.y, self.quaternion.quaternion.z, self.quaternion.quaternion.w)


    # Send the transform
        self.tf_broadcaster.sendTransform(
            translation,  # Translation
            quaternion,  # Rotation (as a quaternion)
            current_time,  # Timestamp
            'base_link',  # Child frame ID
            'odom'  # Parent frame ID
        )


if __name__ == '__main__':
    publisher = OdometryPublisher()

    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        publisher.publish_odometry()
        rate.sleep()
