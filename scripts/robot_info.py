#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class RobotInfoNode:
    def __init__(self):
        rospy.init_node('robot_info_node', anonymous=True)

        print("\n" + "="*50)
        print("  COLOUR SORTING ROBOT - ROS INFO NODE")
        print("="*50)

        # ===== SUBSCRIBERS =====
        print("\n📥 SUBSCRIBERS ACTIVE:")
        print("-"*50)

        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        print("  ✅ /scan          → LaserScan  (LiDAR sensor)")

        rospy.Subscriber('/camera/image_raw', Image, self.camera_callback)
        print("  ✅ /camera/image_raw → Image   (RGB Camera)")

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        print("  ✅ /odom          → Odometry   (Robot position)")

        rospy.Subscriber('/detected_colour', String, self.colour_callback)
        print("  ✅ /detected_colour → String   (Colour detection)")

        # ===== PUBLISHERS =====
        print("\n📤 PUBLISHERS ACTIVE:")
        print("-"*50)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        print("  ✅ /cmd_vel       → Twist      (Robot movement)")

        self.status_pub = rospy.Publisher('/robot_status', String, queue_size=10)
        print("  ✅ /robot_status  → String     (Robot status)")

        print("\n" + "="*50)
        print("  LIVE DATA FEED:")
        print("="*50)

        # State variables
        self.lidar_min      = 0.0
        self.lidar_ranges   = 0
        self.robot_x        = 0.0
        self.robot_y        = 0.0
        self.robot_z        = 0.0
        self.detected_colour = "NONE"
        self.camera_width   = 0
        self.camera_height  = 0
        self.msg_count      = 0

        self.rate = rospy.Rate(1)  # Print every 1 second

    # ===== CALLBACKS =====
    def lidar_callback(self, msg):
        valid = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        self.lidar_min    = round(min(valid), 2) if valid else 0.0
        self.lidar_ranges = len(msg.ranges)

    def camera_callback(self, msg):
        self.camera_width  = msg.width
        self.camera_height = msg.height
        self.msg_count    += 1

    def odom_callback(self, msg):
        self.robot_x = round(msg.pose.pose.position.x, 3)
        self.robot_y = round(msg.pose.pose.position.y, 3)
        self.robot_z = round(msg.pose.pose.position.z, 3)

    def colour_callback(self, msg):
        self.detected_colour = msg.data

    def run(self):
        while not rospy.is_shutdown():
            print("\n" + "="*50)
            print("  📡 LIVE ROBOT DATA")
            print("="*50)

            print(f"\n  🔴 LIDAR SENSOR:")
            print(f"     Topic        : /scan")
            print(f"     Rays         : {self.lidar_ranges} degrees")
            print(f"     Min Distance : {self.lidar_min} meters")

            print(f"\n  📷 RGB CAMERA:")
            print(f"     Topic        : /camera/image_raw")
            print(f"     Resolution   : {self.camera_width}x{self.camera_height}")
            print(f"     Frames recvd : {self.msg_count}")

            print(f"\n  🎨 COLOUR DETECTION:")
            print(f"     Topic        : /detected_colour")

            if self.detected_colour == "RED":
                print(f"     Detected     : 🔴 RED")
            elif self.detected_colour == "GREEN":
                print(f"     Detected     : 🟢 GREEN")
            elif self.detected_colour == "BLUE":
                print(f"     Detected     : 🔵 BLUE")
            else:
                print(f"     Detected     : ⚪ NONE")

            print(f"\n  📍 ROBOT POSITION:")
            print(f"     Topic        : /odom")
            print(f"     X            : {self.robot_x} m")
            print(f"     Y            : {self.robot_y} m")
            print(f"     Z            : {self.robot_z} m")

            print(f"\n  🚗 MOVEMENT:")
            print(f"     Topic        : /cmd_vel")
            print(f"     Type         : Twist (linear + angular)")

            print(f"\n  📊 STATUS:")
            print(f"     Topic        : /robot_status")
            print(f"     Node         : robot_info_node ✅")

            print("="*50)

            # Publish status
            self.status_pub.publish(f"x:{self.robot_x} y:{self.robot_y} colour:{self.detected_colour}")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = RobotInfoNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
