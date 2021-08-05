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
		self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.search)
		self.status_pub = rospy.Publisher('status', String, queue_size=1)
		self.stop_sub = rospy.Subscriber('reset_yolo', Int32, self.set_stop)
		self.bottle_pub = rospy.Publisher('bottle_visible', Bool, queue_size=1)
		self.stop = 0
		self.bottle_collected = 0
		self.num_detected = 0
		self.bottle_pub.publish(False)
		

	def set_stop(self, num):
		self.stop = num.data
		rospy.loginfo(self.stop)
		self.bottle_pub.publish(False)
	
	def search(self, data):
		for box in data.bounding_boxes:
			if (box.id == 39 or box.id == 12 or box.id == 0 or box.id == 75 or box.id == 10 or box.id == 61) and self.stop <= 2:
				self.num_detected += 1					
				img_mid = 640
				dif = box.xmax - box.xmin
				bottle_mid = box.xmin + (dif / 2)
				threshold = 30
				ang_vel = 0.15
				vel = 0.15

				rospy.loginfo(dif)
				rospy.loginfo(box.Class)
				#rospy.loginfo(rospy.get_time())

				twist = Twist()
				twist.linear.x = 0
				twist.linear.y = 0
				twist.linear.z = 0
				twist.angular.x = 0
				twist.angular.y = 0
				twist.angular.z = 0
				if self.num_detected == 1:
					self.pub.publish(twist)

				if img_mid > bottle_mid + threshold:				
					twist.angular.z = ang_vel
							
				elif img_mid < bottle_mid - threshold:
					twist.angular.z = -ang_vel
				else:
					twist.angular.z = 0
				
				if dif < 100:
					twist.linear.x = vel

				elif dif < 200:
					twist.linear.x = vel - 0.05
				elif dif < 290:
					twist.linear.x = vel-0.1

				else:
					twist.linear.x = 0
					self.stop += 1		
				
				self.bottle_pub.publish(True)
				#rospy.loginfo(box)
				self.pub.publish(twist)
				if self.stop==2:
					self.stop += 1
					twist.angular.x = 0
					twist.angular.y = 0
					twist.angular.z = 0
					twist.linear.x = 0
					twist.linear.y = 0
					twist.linear.z = 0
					self.pub.publish(twist)
					rospy.sleep(2)
					##Pick it up
					self.status_pub.publish("A")
			

def main(args):
	rospy.init_node('task3_yolo', anonymous=True)
	sm = simple_motion()
	#sm.rotate()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("shutting_down")

if __name__ == '__main__':
	main(sys.argv)
