#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class MyNode(object):
    def __init__(self):
        rospy.init_node('my_node', anonymous=True)
        self.get_logger = rospy.loginfo
        self.get_logger("大家好，我是my_node!")
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("camera/image", Image, queue_size=10)
        for device in [11, 16]:
            if self.open_camera(device):
                break

    def open_camera(self, device_id):
        cap = cv2.VideoCapture(device_id)

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not cap.isOpened():
            rospy.logerr(f"Can not find camera at /dev/video{device_id}")
            return False

        rospy.loginfo(f"Camera /dev/video{device_id} is opened successfully.")

        while not rospy.is_shutdown():
            ret, frame = cap.read()

            if not ret:
                rospy.logerr("Failed to grab frame")
                break

            try:
                # Convert the image to a ROS Image message
                image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                # Publish the image message
                self.image_pub.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr("Could not convert the image: %s" % e)

            cv2.imshow(f"Camera /dev/video{device_id}", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        return True

if __name__ == '__main__':
    try:
        my_node = MyNode()
    except rospy.ROSInterruptException:
        pass
