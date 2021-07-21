#!/usr/bin/env python
import rospy
import cv2
import math
from cv_bridge import CvBridge
import gazebo_balloon_detector
import threading
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from copy import deepcopy
import time
from tf.transformations import euler_from_quaternion

# run : rosservice call /mavros/setpoint_velocity/mav_frame "mav_frame: 8" before launching the node


class BalloonKiller(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()

        self.video = gazebo_balloon_detector.Video()
        self.pose = PoseStamped()
        self.vel = TwistStamped()

        self.width = 320

        self.pose_sub = rospy.Subscriber(
            '/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.vel_sub = rospy.Subscriber(
            '/mavros/local_position/velocity_local', TwistStamped, self.vel_cb)

        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    def pose_cb(self, msg):
        self.pose = deepcopy(msg)

    def vel_cb(self, msg):
        self.vel = deepcopy(msg)

    def takeoff(self):
        time.sleep(1)
        des_pose = PoseStamped()
        z = self.pose.pose.position.z
        err = 0.5
        des_pose.pose.position.x = 0.0
        des_pose.pose.position.y = 0.0
        des_pose.pose.position.z = 5.0
        while True:
            z = self.pose.pose.position.z
            self.pose_pub.publish(des_pose)
            rospy.loginfo_throttle(2, "Z = {}".format(z))
            if des_pose.pose.position.z - z < err:
                break
        rospy.loginfo_once("***** Drone Ready to Scan *****")
        time.sleep(1)

    def scanning(self):
        des_vel = TwistStamped()

        x = self.pose.pose.orientation.x
        y = self.pose.pose.orientation.y
        z = self.pose.pose.orientation.z
        w = self.pose.pose.orientation.w
        q = (x, y ,z ,w)
        des_vel.twist.linear.x = 0
        des_vel.twist.linear.y = 0
        des_vel.twist.angular.z = 0.3

        euler =  euler_from_quaternion(q)
        yaw0 = math.degrees(euler[2])
        err = 10
        rounds_counter = 0

        while True:
            self.center = self.video.findBalloon(self.width)
            x = self.pose.pose.orientation.x
            y = self.pose.pose.orientation.y
            z = self.pose.pose.orientation.z
            w = self.pose.pose.orientation.w
            q = (x, y ,z ,w)
            euler = euler_from_quaternion(q)
            yaw = math.degrees(euler[2])

            if self.center is not None:
                rospy.loginfo_once("***** Found Balloon! *****")
                self.balloon_x = self.pose.pose.position.x
                self.balloon_y = self.pose.pose.position.y
                self.balloon_z = self.pose.pose.position.z

                self.balloon_qx = self.pose.pose.orientation.x
                self.balloon_qy = self.pose.pose.orientation.y
                self.balloon_qz = self.pose.pose.orientation.z
                self.balloon_qw = self.pose.pose.orientation.w

                rospy.loginfo_once("Balloon Center: ({},{})".format(self.center[0], self.center[1]))
                break

            elif self.center is None and abs(yaw - yaw0) > err:
                self.vel_pub.publish(des_vel)

            elif self.center is None and abs(yaw - yaw0) < err:
                des_vel.twist.linear.x = rounds_counter
                des_vel.twist.linear.y = rounds_counter
                rounds_counter += 0.01
                des_vel.twist.angular.z = 0.3 + 0.1*rounds_counter
                self.vel_pub.publish(des_vel) 
                rospy.loginfo_throttle(1,"Complete {} Rounds".format(rounds_counter))
                         
            rospy.loginfo_throttle(5, "***** Scanning for Balloon *****")
            self.rate.sleep()

    def hold(self):
        hold_time = 10
        hold_time_start = time.time()

        while time.time() < hold_time + hold_time_start:
            des_pose = PoseStamped()
            des_pose.pose.position.x = self.balloon_x
            des_pose.pose.position.y = self.balloon_y
            des_pose.pose.position.z = self.balloon_z
            des_pose.pose.orientation.x = self.balloon_qx
            des_pose.pose.orientation.y = self.balloon_qy
            des_pose.pose.orientation.z = self.balloon_qz
            des_pose.pose.orientation.w = self.balloon_qw
            self.pose_pub.publish(des_pose)
            self.rate.sleep()

    def tracking(self):
        rospy.loginfo_once("***** Tracking the Balloon *****")

    def run(self):
        self.takeoff()
        while True:
            try:
                self.scanning()
                self.hold()
                self.tracking()
            except KeyboardInterrupt:
                print("Shutting Down the Node")
                break

def main():
    rospy.init_node('balloon_killer', anonymous=True)
    rospy.loginfo_once('***** Initiate Balloon Killer Node *****')
    drone = BalloonKiller()
    drone.start()
    drone.join()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
        pass