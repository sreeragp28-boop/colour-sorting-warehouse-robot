#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class ColourDetector:
    def __init__(self):
        rospy.init_node('colour_detector', anonymous=True)

        self.bridge = CvBridge()

        # Publishers
        self.colour_pub  = rospy.Publisher('/detected_colour', String, queue_size=10)
        self.image_pub   = rospy.Publisher('/camera/colour_detected', Image, queue_size=10)

        # Subscriber
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        # HSV colour ranges
        self.colour_ranges = {
            'RED': [
                (np.array([0,   120,  70]),  np.array([10,  255, 255])),
                (np.array([170, 120,  70]),  np.array([180, 255, 255]))
            ],
            'GREEN': [
                (np.array([36,  100,  70]),  np.array([86,  255, 255]))
            ],
            'BLUE': [
                (np.array([94,  100,  70]),  np.array([126, 255, 255]))
            ]
        }

        self.detected_colour = "NONE"
        rospy.loginfo("Colour Detector Node Started!")

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Convert BGR to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        detected = "NONE"
        largest_area = 500  # minimum area threshold

        for colour_name, ranges in self.colour_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

            for (lower, upper) in ranges:
                mask |= cv2.inRange(hsv, lower, upper)

            # Clean up mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > largest_area:
                    largest_area = area
                    detected = colour_name

                    # Draw bounding box
                    x, y, w, h = cv2.boundingRect(cnt)
                    colour_bgr = {
                        'RED':   (0,   0,   255),
                        'GREEN': (0,   255, 0),
                        'BLUE':  (255, 0,   0)
                    }[colour_name]

                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), colour_bgr, 2)
                    cv2.putText(cv_image, colour_name,
                                (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, colour_bgr, 2)

                    # Draw centre dot
                    cx = x + w // 2
                    cy = y + h // 2
                    cv2.circle(cv_image, (cx, cy), 5, colour_bgr, -1)

        # If colour changed, log and publish
        if detected != self.detected_colour:
            self.detected_colour = detected
            rospy.loginfo(f"Detected colour: {detected}")

        self.colour_pub.publish(detected)

        # Publish annotated image
        try:
            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = ColourDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
