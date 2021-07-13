from imutils.video import VideoStream
import argparse
import cv2
import imutils
import time

class balloonDetector():
	def __init__(self):
		self.greenLower = (29, 86, 6)
		self.greenUpper = (64, 255, 255)

	def videoSrc(self):
		self.ap = argparse.ArgumentParser()
		self.ap.add_argument("-v", "--video", help="path to the (optional) video file")
		self.args = vars(self.ap.parse_args())
	
		if not self.args.get("video", False):
			self.vs = VideoStream(src=0).start()
		else:
			self.vs = cv2.VideoCapture("ball_tracking_example.mp4")
		
	def findBalloon(self):
		self.frame = self.vs.read()
		self.frame = self.frame[1] if self.args.get("video", False) else self.frame

		if self.frame is not None:
			self.frame = imutils.resize(self.frame, width=600)
			_blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)
			_hsv = cv2.cvtColor(_blurred, cv2.COLOR_BGR2HSV)

			_mask = cv2.inRange(_hsv, self.greenLower, self.greenUpper)
			_mask = cv2.erode(_mask, None, iterations=2)
			_mask = cv2.dilate(_mask, None, iterations=2)

			_cnts = cv2.findContours(_mask.copy(), cv2.RETR_EXTERNAL,
				cv2.CHAIN_APPROX_SIMPLE)
			_cnts = imutils.grab_contours(_cnts)
			self.center = None

			if len(_cnts) > 0:
				_c = max(_cnts, key=cv2.contourArea)
				((x, y), _radius) = cv2.minEnclosingCircle(_c)
				M = cv2.moments(_c)
				self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
				if _radius > 10:
					cv2.circle(self.frame, self.center, 5, (0, 0, 255), -1)	

	def guard(self):
		if not self.args.get("video", False):
			self.vs.stop()
		else:
			self.vs.release()

def main():
	detector = balloonDetector()
	detector.videoSrc()
	
	time.sleep(1.0)

	while True:
		detector.findBalloon()

		cv2.imshow("Frame", detector.frame)
		key = cv2.waitKey(1) & 0xFF

		if key == ord("q"):
			detector.guard()
			break

	cv2.destroyAllWindows()

if __name__ == "__main__":
    main()












