#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
import gazebo_balloon_detector
import threading
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from copy import deepcopy

# run : rosservice call /mavros/setpoint_velocity/mav_frame "mav_frame: 8" before launching the node


class BalloonKiller(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()

        self.pose_cb()
        self.vel_cb()
        self.video = gazebo_balloon_detector.Video()

        self.pose_sub = rospy.Subscriber(
            '/mavros/loacl_position/pose', PoseStamped, self.pose_cb)
        self.vel_sub = rospy.Subscriber(
            '/mavros/loacl_position/velocity', TwistStamped, self.vel_cb)

        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    def pose_cb(self, msg):
        self.pose = deepcopy(msg)

    def vel_cb(self, msg):
        self.vel = deepcopy(msg)

    def quaternion_to_euler_angle_vectorized1(w, x, y, z):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = np.degrees(np.arctan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = np.where(t2 > +1.0, +1.0, t2)
        #t2 = +1.0 if t2 > +1.0 else t2

        t2 = np.where(t2 < -1.0, -1.0, t2)
        #t2 = -1.0 if t2 < -1.0 else t2
        Y = np.degrees(np.arcsin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = np.degrees(np.arctan2(t3, t4))

        return Z
    def takeoff(self):
        des_pose = PoseStamped()
        z = self.pose.position.z
        err = 0.5
        des_pose.pose.position.x = 0.0
        des_pose.pose.position.y = 0.0
        des_pose.pose.position.z = 5.0
        while z - des_pose.pose.position.z > err:
            z = self.pose.position.z
            self.pose_pub.publish(des_pose)
        rospy.logerr_once("***** Drone Ready to Scan *****")

    def scanning(self):
        des_vel = TwistStamped()

        x = self.pose.orientation.x
        y = self.pose.orientation.y
        z = self.pose.orientation.z
        w = self.pose.orientation.w
        des_vel.twist.angular.x = 0
        des_vel.twist.angular.y = 0
        des_vel.twist.angular.z = 0.5

        yaw0 = self.quaternion_to_euler_angle_vectorized1(w, x, y, z)
        rounds_counter = 0

        while True:
            self.center = self.video.findBalloon()
            yaw = self.quaternion_to_euler_angle_vectorized1(w, x, y, z)

            if self.center is not None:
                rospy.logerr_once("***** Found Balloon! *****")
                break
            elif self.center is None and abs(yaw - yaw0) < 350:
                self.vel_pub.publish(des_vel)
            elif self.center is None and abs(yaw - yaw0) > 350:
                rounds_counter += 1
                des_vel.twist.angular.x = rounds_counter
                des_vel.twist.angular.y = rounds_counter
                des_vel.twist.angular.z = 0.5
                self.vel_pub.publish(des_vel)              
            rospy.logdebug_throttle(5, "***** Scanning for Balloon *****")

    def run(self):
        self.takeoff()
        while True:
            try:
                self.scanning()
            except KeyboardInterrupt:
                print("Shutting Down the Node")

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