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
		rospy.init_node('straight', anonymous=True)
		self.r = rospy.Rate(50)  # 50hz

		#initialize publisher
		self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=50)
		
		
		#速度
		self.vr = 2 #[km/h]



	def straight(self):
		cmd_vel = Twist()
		cmd_vel.linear.x = self.vr/3.6    #[m/s]
		cmd_vel.linear.y = 0.0
		cmd_vel.linear.z = 0.0
		cmd_vel.angular.x = 0.0
		cmd_vel.angular.y = 0.0
		cmd_vel.angular.z = 0.0
		self.cmdvel_pub.publish(cmd_vel)
		print(cmd_vel)
		self.r.sleep()
		return
			

		
		
	 

if __name__ == '__main__':
    print('Path follower is started...')
    path_publisher = pure_pursuit()
    try:
        while not rospy.is_shutdown():
          path_publisher.straight()
    except KeyboardInterrupt:
      print("Path follower finished!")

