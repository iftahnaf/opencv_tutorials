#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
import gazebo_balloon_detector
import threading
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from copy import deepcopy
import time


class BalloonKiller(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()

        self.video = gazebo_balloon_detector.Video(self)
        self.width = 320
        self.height = 240

        self.pose = PoseStamped()
        self.vel = TwistStamped()

        self.pose_sub = rospy.Subscriber(
            '/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.vel_sub = rospy.Subscriber(
            '/mavros/local_position/velocity_local', TwistStamped, self.vel_cb)

        self.vel_pub = rospy.Publisher(
            '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.pose_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    def __call__(self, data):
        self.center = data

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
        while True:
            self.center = self.video.findBalloon(self.width)
            if self.center is not None:
                rospy.loginfo_once("***** Found Balloon! *****")
                self.balloon = deepcopy(self.pose)
                rospy.loginfo_once("Balloon Center: ({},{})".format(
                    self.center[0], self.center[1]))
                self.video.start()
                break

            else:
                des_vel.twist.linear.x = 0.0
                des_vel.twist.linear.y = 0.0
                des_vel.twist.linear.z = 0.0
                des_vel.twist.angular.x = 0.0
                des_vel.twist.angular.y = 0.0
                des_vel.twist.angular.z = 0.1
                self.vel_pub.publish(des_vel)

            rospy.loginfo_throttle(5, "***** Scanning for Balloon *****")
            self.rate.sleep()

    def hold(self):
        hold_time = 2
        hold_time_start = time.time()
        while time.time() < hold_time + hold_time_start:
            self.pose_pub.publish(self.balloon)
            self.rate.sleep()

    def positioning(self):
        rospy.loginfo_once("***** Tracking the Balloon *****")
        des_vel = TwistStamped()
        tol = 10
        des_vel.twist.angular.x = 0.0
        des_vel.twist.angular.y = 0.0
        des_vel.twist.angular.z = 0.0
        des_vel.twist.linear.x = 0.0
        des_vel.twist.linear.y = 0.0

        if self.center[1] > self.height / 2.0:
            des_vel.twist.linear.z = -0.1
        else:
            des_vel.twist.linear.z = 0.1

        while True:
            try:
                self.vel_pub.publish(des_vel)
                if self.center[1] - self.height/2 < tol:
                    break
            except:
                continue
            self.rate.sleep()

        self.balloon = deepcopy(self.pose)
        self.hold()
        des_vel.twist.linear.z = 0.0

        if self.center[0] < self.width/2.0:
            des_vel.twist.angular.z = 0.1
        else:
            des_vel.twist.angular.z = -0.1

        while True:
            try:
                self.vel_pub.publish(des_vel)
                if self.center[0] - self.width/2.0 < tol:
                    break
            except:
                continue
            self.rate.sleep()

        self.balloon = deepcopy(self.pose)
        self.hold()

    def run(self):
        self.takeoff()
        while True:
            try:
                self.scanning()
                self.hold()
                self.positioning()
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
