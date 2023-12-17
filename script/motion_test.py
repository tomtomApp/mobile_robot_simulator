#! /usr/bin/env python3
import numpy as np
import math
import pandas as pd

# import for ros function
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path, Odometry

#imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion


class pure_pursuit():
	def __init__(self):
		rospy.init_node('motion_test', anonymous=True)
		self.r = rospy.Rate(50)  # 50hz
		self.path_first_flg = False
		self.odom_first_flg = False
		self.position_search_flg = False
		self.imu_first_flg = False
		self.path_num_first_flg = False
		self.last_indx = 0
		self.path_num = 0

		#initialize publisher
		self.posestamped_pub = rospy.Publisher("imu_pose", PoseStamped, queue_size=50)
		self.posestamped_pub = rospy.Publisher("imu_pose", PoseStamped, queue_size=50)
		#initialize subscriber
		rospy.Subscriber("/imu/data", Imu, self.imu_callback)
		rospy.Subscriber("/imu/data", Imu, self.cb_get_odometry_subscriber)
		

	def path_follower(self):
		if self.imu_first_flg == True:
			#print(self.current_yaw_euler)
			#print('follow')
			a = 0

	
	def imu_callback(self, imu_msg):
		ps_msg = PoseStamped()
		ps_msg.header=imu_msg.header
		ps_msg.pose.orientation=imu_msg.orientation
		self.posestamped_pub.publish(ps_msg)
		x = ps_msg.pose.orientation.x
		y = ps_msg.pose.orientation.y
		z = ps_msg.pose.orientation.z
		w = ps_msg.pose.orientation.w

		yaw = self.quaternion_to_euler(Quaternion(x, y, z, w)).z
		self.current_yaw_euler = yaw
		
		yaw = self.quaternion_to_euler(Quaternion(x, y, z, w)).z
		self.current_yaw_euler = yaw + np.pi 
		
		if self.current_yaw_euler > np.pi:
			self.current_yaw_euler = yaw - 2 * np.pi
		elif self.current_yaw_euler < np.pi:
			self.current_yaw_euler = yaw + 2 * np.pi
		
		#print(yaw)
		print('imu', self.current_yaw_euler)
		#print('x:', ps_msg.pose.orientation.x)
		#print('y:', ps_msg.pose.orientation.y)
		#print('z:', np.degrees(yaw))
		self.imu_first_flg = True
	    
	def quaternion_to_euler(self, quaternion):
	    """Convert Quaternion to Euler Angles

	    quarternion: geometry_msgs/Quaternion
	    euler: geometry_msgs/Vector3
	    """
	    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
	    return Vector3(x=e[0], y=e[1], z=e[2])
	    
	def cb_get_odometry_subscriber(self,imu_msg):
		ps_msg = PoseStamped()
		ps_msg.header=imu_msg.header
		ps_msg.pose.orientation=imu_msg.orientation
		self.posestamped_pub.publish(ps_msg)
		x = ps_msg.pose.orientation.x
		y = ps_msg.pose.orientation.y
		z = ps_msg.pose.orientation.z
		w = ps_msg.pose.orientation.w
		e = tf.transformations.euler_from_quaternion((x,y,z,w))
		yaw_euler = e[2]
		self.current_yaw_euler = yaw_euler
		self.odom_first_flg = True
		print('odm:',self.current_yaw_euler)
		#print("odom_sub")
		#print('current_x:',self.current_x)
		#print('current_y:',self.current_y)
		
		
	 

if __name__ == '__main__':
    print('Path follower is started...')
    path_publisher = pure_pursuit()
    try:
        while not rospy.is_shutdown():
          path_publisher.path_follower()
    except KeyboardInterrupt:
      print("Path follower finished!")

