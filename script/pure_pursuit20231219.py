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
from nav_msgs.msg import Path, Odometry


class pure_pursuit():
	def __init__(self):
		rospy.init_node('pure_pursuit2023', anonymous=True)
		self.r = rospy.Rate(50)  # 50hz
		self.path_first_flg = False
		self.odom_first_flg = False
		self.position_search_flg = False
		self.last_indx = 0

		#initialize publisher
		self.cmdvel_pub = rospy.Publisher("/cmd_vel_automatic", Twist, queue_size=50)
		self.lookahed_pub = rospy.Publisher("/lookahed_marker", Marker, queue_size=50)
		self.targetwp_num_pub = rospy.Publisher("/targetwp_num", Int32, queue_size=10)

		#initialize subscriber
		self.path_sub = rospy.Subscriber("/path", Path, self.cb_get_path_topic_subscriber)
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.cb_get_odometry_subscriber)

		#速度
		self.vr = 2 #[km/h]
		self.start = 0
		self.count = 0
		self.times = 0
		self.past = []
		#ロボット座標と注視点との距離の設定
		self.ld_th = 0.4

		self.rand = 0.5
		#self.path_follower()

	def path_follower(self):
		if self.path_first_flg == True and self.odom_first_flg == True:
			if self.start == 0:
				self.start += 1
				self.position0 = [self.path[0][0], self.path[0][1]]
				self.attention = [self.path[1][0], self.path[1][1]]
				#始点と注視点との距離
				ld = np.sqrt(np.power(self.attention[0]-self.position0[0],2)+np.power(self.attention[1]-self.position0[1],2))
				#min_indx = ld.argmin()
				#初期角度
				self.theta = self.current_yaw_euler
				self.alpha =  np.arctan2(self.attention[1]-self.position0[1], self.attention[0]-self.position0[0]) - self.theta
				#print(np.degrees(self.theta))
				#print(np.degrees(self.alpha))
				self.omega = 2 * self.vr * np.sin(self.alpha) / ld
				#print(self.theta)
				#self.R = self.vr / self.omega
			#自己位置
			self.count += 1
			'''
			self.laterX = self.laterX + self.vr * np.cos(self.omega + self.theta)
			self.laterY = self.laterY + self.vr * np.sin(self.omega + self.theta)
			'''
			#print('self.current_x:',self.current_x)
			self.laterX = self.current_x
			self.laterY = self.current_y
			self.theta = self.current_yaw_euler
			print(self.theta)
			self.past.append([self.laterX ,self.laterY])
			#自己位置から目標位置までの距離D
			self.D = np.sqrt(np.power(self.attention[0]-self.laterX,2)+np.power(self.attention[1]-self.laterY,2))

			self.count = 0
			self.position0 = [self.laterX, self.laterY]
			'''
			if self.times > len(self.path)-1:
			self.attention = [self.path[-1][0], self.path[-1][1]]
			'''
			if self.D < self.ld_th:
				imageD = 0
				while(imageD < self.ld_th):
					if self.times < len(self.path)-1:
						self.times += 1
						imageD = np.sqrt(np.power(self.path[self.times][0]-self.laterX,2)+np.power(self.path[self.times][1]-self.laterY,2))
					else:
						break
			#print(self.times)
			self.targetwp_num_pub.publish(self.times)
                       
			#終了条件
			if np.sqrt(np.power(self.laterX-self.path[-1][0],2)+np.power(self.laterY-self.path[-1][1],2)) < 0.3:
				cmd_vel = Twist()
				self.cmdvel_pub.publish(cmd_vel)
				print("終了")
				#self.follow_plot()
				return
			
			self.attention = [self.path[self.times][0], self.path[self.times][1]]
			#始点と注視点との距離
			ld = np.sqrt(np.power(self.attention[0]-self.position0[0],2)+np.power(self.attention[1]-self.position0[1],2))
			self.alpha =  np.arctan2(self.attention[1]-self.position0[1], self.attention[0]-self.position0[0]) - self.theta
			self.omega = 2 * self.vr * np.sin(self.alpha) / ld

			#Set Cmdvel
			cmd_vel = Twist()
			cmd_vel.linear.x = self.vr/3.6    #[m/s]
			cmd_vel.linear.y = 0.0
			cmd_vel.linear.z = 0.0
			cmd_vel.angular.x = 0.0
			cmd_vel.angular.y = 0.0
			cmd_vel.angular.z = self.omega
			self.cmdvel_pub.publish(cmd_vel)

			self.r.sleep()
			return
		
	def transform_to_robot_frame(self, point, target, yaw):
		x, y = target
		dx = (point[0] - x)
		dy = (point[1] - y)
		x_in_robot = dx * math.cos(yaw) - dy * math.sin(yaw) 
		y_in_robot = dy * math.cos(yaw) + dx * math.sin(yaw)
		return x_in_robot, y_in_robot

	def cb_get_path_topic_subscriber(self, msg):
		if self.path_first_flg != True and self.odom_first_flg ==True:
			self.prePath = np.array([[msg.poses[indx].pose.position.x, msg.poses[indx].pose.position.y] for indx in range(len(msg.poses))])
			point = [self.prePath[0][0], self.prePath[0][1]]
			self.path = []
			yaw = self.initYaw
			for i in range(len(self.prePath)):
				target = [self.prePath[i][0], self.prePath[i][1]] 
				dump = self.transform_to_robot_frame(point, target, yaw)
				self.path.append([dump[0], dump[1]])
			self.path = np.array(self.path)
			self.path_first_flg = True
			print('self.path[0][0]:',self.path[0][0])
			'''
			self.path_x_np = np.zeros([len(msg.poses)])
			self.path_y_np = np.zeros([len(msg.poses)])
			self.path_st_np = np.zeros([len(msg.poses)])
			self.pass_flg_np = np.zeros([len(msg.poses)])
			last_x = 0.0
			last_y = 0.0
			last_st = 0.0
			for indx in range(len(msg.poses)):
			self.path_x_np[indx] = msg.poses[indx].pose.position.x
			self.path_y_np[indx] = msg.poses[indx].pose.position.y
			self.path_st_np[indx] = last_st + math.sqrt((self.path_x_np[indx]-last_x)**2 + (self.path_y_np[indx]-last_y)**2)
			last_x = self.path_x_np[indx]
			last_y = self.path_y_np[indx]
			last_st = self.path_st_np[indx]

			#print("path(first)")
			'''

	def cb_get_odometry_subscriber(self,msg):
		self.current_x = msg.pose.pose.position.x
		self.current_y = msg.pose.pose.position.y
		e = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
		yaw_euler = e[2] + self.rand
		if self.odom_first_flg == False:
			self.initYaw = yaw_euler
		self.current_yaw_euler = yaw_euler - self.initYaw
		print(self.current_yaw_euler)
		self.odom_first_flg = True
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

