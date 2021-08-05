#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes
import sys

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class simple_motion:
	def __init__(self):
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.arm_pub = rospy.Publisher('move_arm', String, queue_size=1)
		self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.search)
		self.stop = 0
		self.bottle_collected = 0
		rospy.sleep(1)
		self.rotate(90)


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
			rospy.loginfo(twist)
			twist.angular.z = -0.25
			self.pub.publish(twist)
			rospy.sleep(7)
			twist.angular.z = 0
		elif angle == 180:
			twist.angular.z = 0.50
			self.pub.publish(twist)
			rospy.sleep(7)
			twist.angular.z = 0
		self.pub.publish(twist)
		
		
	def search(self, data):
		
		for box in data.bounding_boxes:
			if (box.id == 39 or box.id == 12 or box.id == 0 or box.id == 75 or box.id == 10 or box.id == 61) and self.stop <= 2:
				img_mid = 640
				dif = box.xmax - box.xmin
				bottle_mid = box.xmin + (dif / 2)
				threshold = 30
				ang_vel = 0.15
				vel = 0.15

				rospy.loginfo(box)
				rospy.loginfo(rospy.get_time())

				twist = Twist()
				twist.linear.x = 0
				twist.linear.y = 0
				twist.linear.z = 0
				twist.angular.x = 0
				twist.angular.y = 0
				twist.angular.z = 0

				if img_mid > bottle_mid + threshold:				
					twist.angular.z = ang_vel
							
				elif img_mid < bottle_mid - threshold:
					twist.angular.z = -ang_vel
				else:
					twist.angular.z = 0
				
				if dif < 100:
					twist.linear.x = vel-0.05

				elif dif < 200:
					twist.linear.x = vel - 0.05
				elif dif < 260:
					twist.linear.x = vel-0.1

				else:
					twist.linear.x = 0
					self.stop += 1		
				
				#rospy.loginfo(box)
				self.pub.publish(twist)
				if self.stop==2:
					twist.angular.x = 0
					twist.angular.y = 0
					twist.angular.z = 0
					twist.linear.x = 0
					twist.linear.y = 0
					twist.linear.z = 0
					self.pub.publish(twist)
					rospy.sleep(0.5)
					##Pick it up
					self.arm_pub.publish("2")
					self.stop += 1
					rospy.sleep(15)
					goal = MoveBaseGoal()
					##Bringin back
					goal.target_pose.header.frame_id = "map"
					goal.target_pose.header.stamp = rospy.Time.now()
					goal.target_pose.pose.position.x = 0.3
					goal.target_pose.pose.position.y = -0.05
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
							self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.search)
							rospy.sleep(1)
							self.stop =0
							self.rotate(90)

def main(args):
	rospy.init_node('task1', anonymous=True)
	sm = simple_motion()
	#sm.rotate()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("shutting_down")

if __name__ == '__main__':
	main(sys.argv)
