#!/usr/bin/env python
# -*- coding: utf-8 -*-

import string
from matplotlib.transforms import Transform
import rospy
import numpy as np
import tf
from pyproj import Proj
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from math import pi

from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class LL2UTMConverter:
    def __init__(self, zone=52) :

        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.imu_sub = rospy.Subscriber("/imu",Imu, self.imu_callback)
        self.state_sub = rospy.Subscriber("/state",String, self.state_callback)

        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)
        self.gps_pub = rospy.Publisher('/gps1',GPSMessage,queue_size =1)

        self.x, self.y = None, None
        self.roll,self.pitch,self.yaw = None,None,None
        self.state = None
        self.gps1_msg=GPSMessage()
        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id='odom'
        self.odom_msg.child_frame_id='base_link'
        
        self.proj_UTM = Proj(proj='utm', zone=52, ellps = 'WGS84', preserve_units=False)

    def state_callback(self, state_msg):
        self.state = state_msg

    def navsat_callback(self, gps_msg):
        
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        self.alt = gps_msg.altitude

        # self.lat = 0
        # self.lon = 0
        # self.alt = 0

        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        self.gps1_msg.latitude = self.lat
        self.gps1_msg.longitude = self.lon
        self.gps1_msg.altitude = self.alt

        self.convertLL2UTM()

        br = tf.TransformBroadcaster()
        br.sendTransform((self.x, self.y, 0.),
                        tf.transformations.quaternion_from_euler(0,0,self.yaw),
                        rospy.Time.now(),
                        "odom",
                        "map")
        
        btr = tf.TransformBroadcaster()
        btr.sendTransform((self.x, self.y, 0.),
                        tf.transformations.quaternion_from_euler(0,0,self.yaw),
                        rospy.Time.now(),
                        "ObjectInfo",
                        "map")

        brr = tf.TransformBroadcaster()
        brr.sendTransform((0, 0, 1.25),
                        tf.transformations.quaternion_from_euler(0,0,0.),
                        rospy.Time.now(),
                        "lidar",
                        "odom")
        
        brrr = tf.TransformBroadcaster()
        brrr.sendTransform((0., 0., 0.),
                        tf.transformations.quaternion_from_euler(0,0,0.),
                        rospy.Time.now(),
                        "local",
                        "odom")

        utm_msg = Float32MultiArray()
        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0
        self.odom_pub.publish(self.odom_msg)

        utm_msg.data = [self.x, self.y]
        # print(utm_msg.data)
        # print('lat : ', self.lat)
        # print('lon : ', self.lon)

        self.gps_pub.publish(self.gps1_msg)

    def imu_callback(self, data):
        self.is_imu=True
        self.odom_msg.pose.pose.orientation.x = data.orientation.x
        self.odom_msg.pose.pose.orientation.y = data.orientation.y
        self.odom_msg.pose.pose.orientation.z = data.orientation.z
        self.odom_msg.pose.pose.orientation.w = data.orientation.w
        quaternion=(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w)
        self.roll,self.pitch,self.yaw=euler_from_quaternion(quaternion)
        self.prev_time=rospy.get_rostime()

    def convertLL2UTM(self):

        xy_zone = self.proj_UTM(self.lon, self.lat)

        self.x = xy_zone[0] - self.e_o
        self.y = xy_zone[1] - self.n_o


if __name__ == '__main__':

    rospy.init_node('gps_parser', anonymous=True)

    gps_parser = LL2UTMConverter()

    rospy.spin()
        
