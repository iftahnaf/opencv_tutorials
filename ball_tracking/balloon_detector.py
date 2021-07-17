#!/usr/bin/env python

from imutils.video import VideoStream
import argparse
import cv2
import imutils
import time
import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst

class Video():
    """BlueRov video capture class constructor
    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """

    def __init__(self, port=5600):
        """Summary
        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]
        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array
        Args:
            sample (TYPE): Description
        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame
        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available
        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK



class balloonDetector():
	def __init__(self):
		self.greenLower = (29, 86, 6)
		self.greenUpper = (64, 255, 255)

	def videoSrc(self):
		self.ap = argparse.ArgumentParser()
		self.ap.add_argument("-v", "--video", help="path to the (optional) video file")
		self.args = vars(self.ap.parse_args())
	
		if not self.args.get("video", False):
			self.vs = Video()
			self.flag = 1
		else:
			self.vs = cv2.VideoCapture("ball_tracking_example.mp4")
			self.flag = 0
		
		print("Video Source : {}".format(self.vs))
		
	def findBalloon(self):
		if self.flag == 0:
			self.frame = self.vs.read()
			self.frame = self.frame[1] if self.args.get("video", False) else self.frame
		else:
			self.frame = self.vs.frame()

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