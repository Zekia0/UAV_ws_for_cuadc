#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def talker():
    rospy.init_node('video_publisher', anonymous=True)
    
    pub = rospy.Publisher('/video/image', Image, queue_size=10)
    
    rate = rospy.Rate(30) 
    
    cap = cv2.VideoCapture('/dev/video11')
    
    if not cap.isOpened():
        rospy.logerr("Could not open video device")
        return
    
    bridge = CvBridge()
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        
        if ret:
            try:
                msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                pub.publish(msg)
            except CvBridgeError as e:
                rospy.logerr("Could not convert image: {0}".format(e))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
