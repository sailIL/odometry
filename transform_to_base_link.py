#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

def create_tf_transforms():
    rospy.init_node('tf_transforms')
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Create the first transform: laser_scan => base_link
    # transform_1 = geometry_msgs.msg.TransformStamped()
    # transform_1.header.frame_id = "odom"  
    # transform_1.header.stamp = rospy.Time.now()
    # transform_1.child_frame_id = 'base_link'
    # transform_1.transform.translation.x = 0.0  
    # transform_1.transform.translation.y = 0.0
    # transform_1.transform.translation.z = 0.0
    # transform_1.transform.rotation.x = 0.0  
    # transform_1.transform.rotation.y = 0.0
    # transform_1.transform.rotation.z = 0.0
    # transform_1.transform.rotation.w = 1.0

    # Create the second transform: base_link => odom
    transform_2 = geometry_msgs.msg.TransformStamped()
    transform_2.header.frame_id = 'base_link'
    transform_2.header.stamp = rospy.Time.now()
    transform_2.child_frame_id = "lidar_link"
    transform_2.transform.translation.x = 1.1 
    transform_2.transform.translation.y = 0.0
    transform_2.transform.translation.z = 0.18
    transform_2.transform.rotation.x = 0.0  
    transform_2.transform.rotation.y = 0.0
    transform_2.transform.rotation.z = 0.0
    transform_2.transform.rotation.w = 1.0

    # transform_3 = geometry_msgs.msg.TransformStamped()
    # transform_3.header.frame_id = 'base_link'
    # transform_3.header.stamp = rospy.Time.now()
    # transform_3.child_frame_id = "imu_link"
    # transform_3.transform.translation.x = 0.0 
    # transform_3.transform.translation.y = 0.0
    # transform_3.transform.translation.z = 0.0
    # transform_3.transform.rotation.x = 0.0  
    # transform_3.transform.rotation.y = 0.0
    # transform_3.transform.rotation.z = 0.0
    # transform_3.transform.rotation.w = 1.0

    tf_broadcaster.sendTransform([ transform_2])
    rospy.spin()

if __name__ == '__main__':
    try:
        create_tf_transforms()
    except rospy.ROSInterruptException:
        pass
