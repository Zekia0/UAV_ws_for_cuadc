#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from uav_pkg.msg import deta_xy  

class CircleDetector:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/video/image", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/processed_image", Image, queue_size=10)
        self.deta_pub  = rospy.Publisher("/deta_xy", deta_xy, queue_size=10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv2.rectangle(cv_image, (318, 238), (322, 242), (0, 0, 255), -1)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (9, 9), 2, 2)
        
        edges = cv2.Canny(blur, 50, 150)
        
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, 1000, param1=15, param2=17, minRadius=9, maxRadius=25)
        
        deta_info = deta_xy() 
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            smallest_radius_circle = None
            min_radius = float('inf')
            for (x, y, r) in circles:
                if r < min_radius:
                    min_radius = r
                    smallest_radius_circle = (x, y, r)
            
            if smallest_radius_circle is not None:
                (x, y, r) = smallest_radius_circle
                cv2.circle(cv_image, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(cv_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                rospy.loginfo("x=%d,y=%d, r=%d", x, y, r)
                deta_info.deta_x = 320 - x
                deta_info.deta_y = 240 - y

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
        self.deta_pub.publish(deta_info)

if __name__ == '__main__':
    rospy.init_node('circle_1')
    circle_1 = CircleDetector()
    rospy.spin()
