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
        self.height = 240

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

        euler =  euler_from_quaternion(q)
        yaw0 = math.degrees(euler[2])
        err = 10

        des_vel.twist.linear.x = 0
        des_vel.twist.angular.z = 0.3

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
                self.balloon = deepcopy(self.pose)
                rospy.loginfo_once("Balloon Center: ({},{})".format(self.center[0], self.center[1]))
                break

            elif self.center is None and abs(yaw - yaw0) > err:
                self.vel_pub.publish(des_vel)

            elif self.center is None and abs(yaw - yaw0) < err:
                des_vel.twist.linear.x = 0.01*rounds_counter
                des_vel.twist.angular.z = 0.3 + 0.01*rounds_counter
                self.vel_pub.publish(des_vel) 
                rospy.loginfo_throttle(1,"Complete {} Rounds".format(int(rounds_counter)))
                rounds_counter += 1
                         
            rospy.loginfo_throttle(5, "***** Scanning for Balloon *****")
            self.rate.sleep()

    def hold(self):
        hold_time = 10
        hold_time_start = time.time()
        while time.time() < hold_time + hold_time_start:
            self.pose_pub.publish(self.balloon)
            self.rate.sleep()

    def tracking(self):
        rospy.loginfo_once("***** Tracking the Balloon *****")
        des_vel = TwistStamped()
        tol = 20
        if self.center[1] > self.height / 2.0:
            des_vel.twist.linear.z = 0.5
            self.vel_pub.publish


        # while self.center < self.width/2.0:
        #     
        #     des_vel.twist.z

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