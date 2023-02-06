#!/usr/bin/env python3


  #Project Name: sentinel_drone
  #Author: Saail
  #Maintainer: ROS team
  #Copyright (c) 2023 eYantra IITB 


#converting udp vide frames to ros topic
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import cv2, socket, numpy, pickle  
import os

# def publish_message():
# 	pub = rospy.Publisher('video_frames', Image, queue_size=10)
# 	rospy.init_node('video_pub_py', anonymous=True)
# 	rate = rospy.Rate(10) # 10hz
# 	cap = cv2.VideoCapture(0)
# 	br = CvBridge()
# 	while not rospy.is_shutdown():
# 		ret, frame = cap.read()
# 		if ret == True:
# 			rospy.loginfo('publishing video frame')
# 			pub.publish(br.cv2_to_imgmsg(frame))
# 		rate.sleep()


# if __name__ == '__main__':
# 	try:
# 		publish_message()
# 	except rospy.ROSInterruptException:
# 		pass


def publish_message():

	s=socket.socket(socket.AF_INET , socket.SOCK_DGRAM)  # Gives UDP protocol to follow

	#ip="192.168.0.137"   # Server public IP
	ip=os.environ['ROS_IP']
	port=2003         # Server Port Number to identify the process that needs to recieve or send packets
	s.bind((ip,port))     # Bind the IP:port to connect 

	# In order to iterate over block of code as long as test expression is true






	pub = rospy.Publisher('video_frames', Image, queue_size=10)
	rospy.init_node('video_pub_py', anonymous=True)
	rate = rospy.Rate(30) # 10hz
	# cap = cv2.VideoCapture(0)2
	br = CvBridge()
	while not rospy.is_shutdown():
		x=s.recvfrom(100000000)    # Recieve byte code sent by client using recvfrom
		clientip = x[1][0]         # x[1][0] in this client details stored,x[0][0] Client message Stored
		data=x[0]                  # Data sent by client
		data=pickle.loads(data)    # All byte code is converted to Numpy Code 
		data = cv2.imdecode(data, cv2.IMREAD_COLOR)  # Decode 
	# 	cv2.imshow('my pic', data) # Show Video/Stream
	# 	if cv2.waitKey(10) == 13:  # Press Enter then window will close
	# 		break
	# cv2.destroyAllWindows() 


		# ret, frame = data.read()
		# if ret == True:
		rospy.loginfo('publishing video frame')
		pub.publish(br.cv2_to_imgmsg(data))
		rate.sleep()


if __name__ == '__main__':
	try:
		publish_message()
	except rospy.ROSInterruptException:
		pass
