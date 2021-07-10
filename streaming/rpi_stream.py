import cv2
import rospy
from sensor_msgs.msg import Image
import cv_bridge

class PublishImage():
    def __init__(self):
        self.image = Image()
        self.bridge = cv_bridge()

    def publisher():
        return rospy.Publisher('/drone/camera', Image, queue_size=10)

    def captureImage(self):
        cap = cv2.VideoCapture("rtp://127.0.0.1:5000") # zero instead of one

        while True:
            ret, frame = cap.read()

            if not ret: # exit loop if there was problem to get frame to display
                break
            
            self.ros_image = cv_bridge.CvBridge.cv2_to_imgmsg(frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

def main():
    rospy.init_node('rpi_stream', anonymous=True)
    rospy.loginfo_once("***** Starting rpi_stream node *****")
    camera = PublishImage()
    camera_pub = camera.publisher()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        camera.captureImage()
        camera_pub.publish(camera.ros_image)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
