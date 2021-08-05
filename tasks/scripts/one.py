#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes
import sys

class simple_motion:
	def __init__(self):
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.arm_pub = rospy.Publisher('move_arm', String, queue_size=1)
		self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.search)
		self.rotate = True
		self.stop = 0

	def search(self, data):
		#rospy.loginfo(data)
		#rospy.loginfo(twist)
		for box in data.bounding_boxes:
			if (box.id == 39 or box.id == 12 or box.id == 0 or box.id == 75 or box.id == 10 or box.id == 61) and self.stop <= 2:
				img_mid = 640
				dif = box.xmax - box.xmin
				bottle_mid = box.xmin + (dif / 2)
				threshold = 30
				ang_vel = 0.15
				vel = 0.15

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
					twist.linear.x = vel

				elif dif < 200:
					twist.linear.x = vel - 0.05
				elif dif < 290:
					twist.linear.x = vel-0.1

				else:
					twist.linear.x = 0
					self.stop += 1		
				rospy.loginfo(dif)
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
					self.arm_pub.publish("g")
					
					self.stop += 1
			
def main(args):
	rospy.init_node('task1', anonymous=True)
	sm = simple_motion()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("shutting_down")

if __name__ == '__main__':
	main(sys.argv)
