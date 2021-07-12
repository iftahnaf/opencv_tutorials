#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class PublishImage():
    def __init__(self):
        self.image = Image()
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher('/camera', Image, queue_size=10)

    def gstreamer_pipeline(self):
        self.capture_width=1280,
        self.capture_height=720,
        self.display_width=1280,
        self.display_height=720,
        self.framerate=60,
        self.flip_method=0

        return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int){}, height=(int){}, "
            "format=(string)NV12, framerate=(fraction){}/1 ! "
            "nvvidconv flip-method={} ! "
            "video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            .format(
                self.capture_width,
                self.capture_height,
                self.framerate,
                self.flip_method,
                self.display_width,
                self.display_height,
            )
        )


    def captureImage(self):
        cap = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)# zero instead of one

        ret, frame = cap.read()

        if not ret: # exit loop if there was problem to get frame to display
            rospy.loginfo_once("Error in Image")
        
        self.ros_image = self.bridge.cv2_to_imgmsg(frame)

        cap.release()
        cv2.destroyAllWindows()

def main():
    rospy.init_node('rpi_stream', anonymous=True)
    rospy.loginfo_once("***** Starting rpi_stream node *****")
    camera = PublishImage()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        camera.captureImage()
        camera.publisher.publish(camera.ros_image)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
