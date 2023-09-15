#!/usr/bin/env python

from roslib import message
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import math

class PointCloudToLaserScan:

    def __init__(self):


        ####### point cloud init #######
        z = 0.1
        delta_z = 0.1
        self.selected_min_z_layer = z   #this value determines the z layer we want to transform into laser scan
        self.selected_max_z_layer = z + delta_z  #this value determines the z layer we want to transform into laser scan
        self.data = None
        ################################


        ####### laser scan init ##########
        self.laser_scan_length = 2000 # gmapping cant work with laser scan lists longer than 2048. 2000 was taken as a safety measure.
        self.laser_scan = LaserScan()
        self.laser_scan.header.frame_id = "lidar_link"
        self.laser_scan.angle_min = (-23 * np.pi/ 72)  #based on lidar  "innoviz one" specs from the website
        self.laser_scan.angle_max = (23 * np.pi/ 72 )    #based on lidar  "innoviz one" specs from the website
        self.laser_scan.angle_increment = (self.laser_scan.angle_max - self.laser_scan.angle_min) / self.laser_scan_length
        self.laser_scan.range_min = 0.1    #based on lidar  "innoviz one" specs from the website
        self.laser_scan.range_max = 250.0    #based on lidar  "innoviz one" specs from the website
        ###################################


        ######## ros init #############
        rospy.Subscriber('invz_summation_reflection', PointCloud2 , self.point_cloud_callback)
        self.laser_scan_pub = rospy.Publisher("pointcloud_to_laserscan", LaserScan, queue_size=1)
        #################################


        

    def map_values(self,value, from_low, from_high, to_low, to_high):  #same as arduino "map" function but always returns int value
        data = int((value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low)
        if data < to_low :
            data = to_low

        elif data > to_high:
            data =  to_high

        return data


    def point_cloud_callback(self, point_cloud_data):

        self.data = point_cloud_data



    def convert_to_laser_scan(self):
        while not rospy.is_shutdown():
            if self.data is not None:
                range_list = []
                publish_list = [0] * self.laser_scan_length
                for point in pc2.read_points(self.data, field_names=("x", "y", "z"), skip_nans=True):   
                    x, y, z = point
                    if (self.selected_min_z_layer <= z <= self.selected_max_z_layer):
                        angle = math.atan2(y, x)
                        laser_range = np.sqrt(x**2 + y**2 + z**2)
                        publish_list_index = self.map_values(angle, self.laser_scan.angle_min, self.laser_scan.angle_max, 0, (self.laser_scan_length-1))
                        publish_list[publish_list_index] = laser_range

                self.laser_scan.ranges = publish_list
                self.laser_scan.header = self.data.header
                self.laser_scan_pub.publish(self.laser_scan)
                self.data = None
    
    
if __name__ == '__main__':
    rospy.init_node("point_cloud_to_laser_scan")
    pc_to_ls = PointCloudToLaserScan()
    pc_to_ls.convert_to_laser_scan()
    rospy.spin()



     

        

