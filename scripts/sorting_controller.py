#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState

class SortingController:
    def __init__(self):
        rospy.init_node('sorting_controller', anonymous=True)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry,  self.odom_callback)

        # Gazebo services
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Robot state
        self.robot_x        = 0.0
        self.robot_y        = 0.0
        self.robot_yaw      = 0.0
        self.front_distance = 999.0

        # Attached package
        self.attached_package = None

        # Package list: (model_name, pickup_x, pickup_y, colour, bin_x, bin_y)
        self.packages = [
            ('pkg_red_1',   -1.5,  1.0, 'RED',   1.8,  2.0),
            ('pkg_green_1', -1.5,  0.0, 'GREEN', 1.8,  0.0),
            ('pkg_blue_1',  -1.5, -1.0, 'BLUE',  1.8, -2.0),
            ('pkg_red_2',   -2.5,  1.5, 'RED',   1.8,  2.0),
            ('pkg_green_2', -2.5, -0.5, 'GREEN', 1.8,  0.0),
            ('pkg_blue_2',  -2.5, -1.5, 'BLUE',  1.8, -2.0),
        ]

        self.rate = rospy.Rate(10)
        rospy.loginfo("=== Sorting Controller Ready ===")
        rospy.sleep(3.0)

    # ===== CALLBACKS =====
    def scan_callback(self, msg):
        ranges    = msg.ranges
        num       = len(ranges)
        front_arc = ranges[0:30] + ranges[num-30:num]
        valid     = [r for r in front_arc if not math.isnan(r) and not math.isinf(r)]
        self.front_distance = min(valid) if valid else 999.0

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # ===== MOVEMENT =====
    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def move_to_goal(self, goal_x, goal_y, tolerance=0.30):
        rospy.loginfo(f"  Moving to ({goal_x:.2f}, {goal_y:.2f})")
        
        # Timeout to avoid getting stuck forever
        start_time = rospy.Time.now()
        timeout    = rospy.Duration(30.0)

        while not rospy.is_shutdown():

            # Timeout check
            if rospy.Time.now() - start_time > timeout:
                rospy.logwarn("  Timeout! Moving on...")
                self.stop_robot()
                return False

            dx   = goal_x - self.robot_x
            dy   = goal_y - self.robot_y
            dist = math.sqrt(dx**2 + dy**2)

            if dist < tolerance:
                self.stop_robot()
                rospy.loginfo("  Reached goal!")
                return True

            # Obstacle stop only when no package attached
            if self.attached_package is None and self.front_distance < 0.35:
                self.stop_robot()
                rospy.sleep(0.3)
                continue

            angle_to_goal = math.atan2(dy, dx)
            angle_diff    = angle_to_goal - self.robot_yaw
            while angle_diff >  math.pi: angle_diff -= 2 * math.pi
            while angle_diff < -math.pi: angle_diff += 2 * math.pi

            twist = Twist()
            if abs(angle_diff) > 0.15:
                # Rotate toward goal
                twist.angular.z = 1.0 * angle_diff
                twist.linear.x  = 0.1
            else:
                # Drive forward fast
                twist.linear.x  = min(0.5, 0.6 * dist)
                twist.angular.z = 0.4 * angle_diff

            self.cmd_vel_pub.publish(twist)

            # Move attached package with robot
            if self.attached_package:
                self.move_package_with_robot(self.attached_package)

            self.rate.sleep()

        return False

    # ===== ATTACH / DETACH =====
    def attach_package(self, model_name):
        rospy.loginfo(f"  Attaching {model_name} to robot...")
        self.attached_package = model_name
        rospy.sleep(0.5)

    def detach_package(self, model_name, bin_x, bin_y):
        rospy.loginfo(f"  Placing {model_name} into bin...")
        state = ModelState()
        state.model_name      = model_name
        state.reference_frame = 'world'
        state.pose.position.x = bin_x + 0.3
        state.pose.position.y = bin_y
        state.pose.position.z = 0.35
        state.pose.orientation.w = 1.0
        try:
            self.set_model_state(state)
            rospy.loginfo(f"  {model_name} placed in bin!")
        except Exception as e:
            rospy.logerr(f"  Detach error: {e}")
        self.attached_package = None
        rospy.sleep(0.5)

    def move_package_with_robot(self, model_name):
        state = ModelState()
        state.model_name      = model_name
        state.reference_frame = 'world'
        state.pose.position.x = self.robot_x
        state.pose.position.y = self.robot_y
        state.pose.position.z = 0.5
        state.pose.orientation.w = 1.0
        try:
            self.set_model_state(state)
        except:
            pass

    # ===== PICK SEQUENCE =====
    def pick_package(self, model_name):
        rospy.loginfo(f">>> PICKING {model_name}")
        rospy.loginfo("  Lowering arm...")
        rospy.sleep(1.2)
        self.attach_package(model_name)
        rospy.loginfo("  Raising arm with package!")
        rospy.sleep(1.0)
        rospy.loginfo(f">>> {model_name} PICKED!")

    # ===== DROP SEQUENCE =====
    def drop_package(self, model_name, bin_x, bin_y):
        rospy.loginfo(f">>> DROPPING {model_name}")
        rospy.loginfo("  Lowering arm to bin...")
        rospy.sleep(1.0)
        self.detach_package(model_name, bin_x, bin_y)
        rospy.loginfo("  Arm raised!")
        rospy.sleep(0.8)
        rospy.loginfo(f">>> {model_name} SORTED! ✓")

    # ===== MAIN SORTING LOOP =====
    def run(self):
        rospy.loginfo("=== SORTING MISSION START ===")

        for i, (model_name, px, py, colour, bx, by) in enumerate(self.packages):
            rospy.loginfo(f"\n{'='*40}")
            rospy.loginfo(f"Package {i+1}/6 : {model_name}")
            rospy.loginfo(f"Colour        : {colour}")
            rospy.loginfo(f"Pickup        : ({px}, {py})")
            rospy.loginfo(f"Bin           : ({bx}, {by})")
            rospy.loginfo(f"{'='*40}")

            # STEP 1 - Navigate to package
            rospy.loginfo(f">> [1/4] Going to {colour} package...")
            self.move_to_goal(px, py, tolerance=0.35)

            # STEP 2 - Pick package
            rospy.loginfo(f">> [2/4] Picking package...")
            self.pick_package(model_name)

            # STEP 3 - Navigate to bin
            rospy.loginfo(f">> [3/4] Going to {colour} bin...")
            self.move_to_goal(bx, by, tolerance=0.60)

            # STEP 4 - Drop package
            rospy.loginfo(f">> [4/4] Dropping into bin...")
            self.drop_package(model_name, bx, by)

            # Return to center
            rospy.loginfo(">> Returning to center...")
            self.move_to_goal(0.0, 0.0, tolerance=0.30)

            rospy.loginfo(f">> Package {i+1} complete!")
            rospy.sleep(0.3)

        rospy.loginfo("\n" + "="*40)
        rospy.loginfo("=== ALL 6 PACKAGES SORTED! ===")
        rospy.loginfo("===   MISSION COMPLETE!     ===")
        rospy.loginfo("="*40)
        self.stop_robot()

if __name__ == '__main__':
    try:
        controller = SortingController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
