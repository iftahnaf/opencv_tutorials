# General
OpenCV tests with Raspberry Pi and Jetson nano

# Table Of Contents:

1. [Installation](#installation)
2. [Streaming From Machine](#streaming-from-machine)
3. [Examples](#examples)


# Installation:
## Gstreamer Installation command:

`sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio`

## Open CV Installation:

`sudo apt-get install python-opencv`

## Access to the camera:

`sudo chmod 777 /dev/video0`

# Streaming From Machine:

Stream from Raspberry Pi or Jetson nano

## On the machine, run the following pipeline to stream from machine to the PC:

*May need to change the IP address or port.*

### Raspberry Pi:

`gst-launch-1.0 -v v4l2src ! video/x-raw,format=YUY2,width=640,height=480 ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.15 port=5000`

### Jetson Nano:

`gst-launch-1.0 nvarguscamerasrc ! nvvidconv ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.15 port=5000 auto-multicast=true`

## **Optional** - Stream to the PC and to a local OpenCV - Python script (Raspberry Pi only):

`gst-launch-1.0 -v v4l2src device=/dev/video0 ! "image/jpeg,width=800,height=600,framerate=30/1" ! rtpjpegpay ! tee name=t ! queue max-size-time=10000000 ! udpsink sync=false host=192.168.1.10 port=5000 t. ! queue max-size-time=10000000 ! udpsink sync=false host=127.0.0.1 port=5000`

## View Streaming from the PC:

`gst-launch-1.0 -v udpsrc port=5000 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=JPEG, payload=26 ! rtpjpegdepay ! jpegdec ! xvimagesink sync=0`

## Accessing to the video stream from OpenCV:
*may need to change the IP address*

`import cv2`

`stream = cv2.VideoCapture("rtp://127.0.0.1:5000")`

`ret,frame = stream.read()`

# Examples

## ball_tracking

Track a green ball through a webcam.

## coins

Image processing Tests.

## streaming

Video streaming from Raspberry Pi V2 Camera Module IMX219 (Rpi or Jetson nano).







