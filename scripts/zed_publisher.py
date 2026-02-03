#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main():
    rospy.init_node('zed_publisher', anonymous=True)

    # Publishers for separate Left and Right frames
    left_pub = rospy.Publisher('/camera_frames', Image, queue_size=10)
    right_pub = rospy.Publisher('/zed/right/image_raw', Image, queue_size=10)

    bridge = CvBridge()

    # Open ZED as a standard UVC camera
    cap = cv2.VideoCapture(0)

    # ZED 720p mode is 2560x720 (Side-by-Side)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        rospy.logerr("Error: Could not open ZED camera. Check USB connection.")
        return

    rate = rospy.Rate(30)
    rospy.loginfo("ZED UVC publisher started. Splitting SBS frames...")

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            continue

        # Split the side-by-side frame in half
        height, width, _ = frame.shape
        half_width = width // 2
        left_img = frame[:, :half_width]
        right_img = frame[:, half_width:]

        try:
            # Convert and publish Left
            left_msg = bridge.cv2_to_imgmsg(left_img, "bgr8")
            left_pub.publish(left_msg)

            # Convert and publish Right
            right_msg = bridge.cv2_to_imgmsg(right_img, "bgr8")
            right_pub.publish(right_msg)

        except CvBridgeError as e:
            rospy.logerr(e)

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass