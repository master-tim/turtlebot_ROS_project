#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import sys

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class simple_motion:
	def __init__(self):
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.arm_pub = rospy.Publisher('move_arm', String, queue_size=1)
		self.status_sub = rospy.Subscriber("status", String, self.bring_home)
		self.stop_pub = rospy.Publisher('reset_yolo', Int32, queue_size=1)
		self.bottle_sub = rospy.Subscriber('bottle_visible', Bool, self.bottle_visible)
		self.bottle_visible = False
		self.bottle_collected = 0
		rospy.sleep(1)
		self.rotate(90)

	def bottle_visible(self, bottle_visible):
		self.bottle_visible = bottle_visible.data

	def bring_home(self, s):
		self.arm_pub.publish("2")
		rospy.sleep(15)
		goal = MoveBaseGoal()
		##Bringin back
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = 0.2
		goal.target_pose.pose.position.y = -0.2
		goal.target_pose.pose.position.z = 0
		goal.target_pose.pose.orientation.x = 0
		goal.target_pose.pose.orientation.y = 0
		goal.target_pose.pose.orientation.z = 1
		goal.target_pose.pose.orientation.w = 0
		res = self.move_to_position(goal)
		##Drop 
		if res:
			self.arm_pub.publish("d")
			self.bottle_collected += 1
			rospy.sleep(15)
			if self.bottle_collected < 2:
				self.rotate(180)
				#rospy.loginfo("publishing 0 back")
				self.stop_pub.publish(0)
				self.rotate(90)
				if not self.bottle_visible and self.bottle_collected < 2:
					self.stop_pub.publish(3)
					rospy.sleep(1)
					##move to center					
					goal = MoveBaseGoal()
					goal.target_pose.header.frame_id = "map"
					goal.target_pose.header.stamp = rospy.Time.now()
					goal.target_pose.pose.position.x = 1.9
					goal.target_pose.pose.position.y = -0.2
					goal.target_pose.pose.position.z = 0
					goal.target_pose.pose.orientation.x = 0
					goal.target_pose.pose.orientation.y = 0
					goal.target_pose.pose.orientation.z = 0
					goal.target_pose.pose.orientation.w = 1
					res = self.move_to_position(goal)
					if res:
						self.rotate(270)


	def move_to_position(self, goal):
		client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		client.wait_for_server()
		client.send_goal(goal)
		wait = client.wait_for_result()
		if not wait:	
			rospy.logerr("Action server not available!")
			return False
    		else:
        		return True

	def rotate(self, angle):
		twist = Twist()
		twist.angular.x = 0
		twist.angular.y = 0
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
		if angle == 90:
			twist.angular.z = 0.25
			self.pub.publish(twist)
			rospy.sleep(3.5)

			#rospy.loginfo(twist)
			twist.angular.z = -0.25
			self.pub.publish(twist)
			rospy.sleep(8)
			twist.angular.z = 0
		elif angle == 180:
			twist.angular.z = 0.50
			self.pub.publish(twist)
			rospy.sleep(7)
			twist.angular.z = 0
		elif angle == 270:
			twist.angular.z = 0.25
			self.pub.publish(twist)
			rospy.sleep(10.5)
			#rospy.loginfo(twist)
			twist.angular.z = -0.25
			self.pub.publish(twist)
			rospy.sleep(21)
			twist.angular.z = 0
		self.pub.publish(twist)
		
		
	

def main(args):
	rospy.init_node('task3_slam', anonymous=True)
	sm = simple_motion()
	#sm.rotate()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("shutting_down")

if __name__ == '__main__':
	main(sys.argv)
