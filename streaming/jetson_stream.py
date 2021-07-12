# import the opencv library
import cv2
  
# define a video capture object
pipeline = "udpsrc port=5000 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=JPEG, payload=26 ! rtpjpegdepay ! jpegdec ! appsink"
vid = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

while(True):
    
    # Capture the video frame
    # by frame
    ret, frame = vid.read()
    assert ret, "No Camera Avialable"
    print(ret)
  
    # Display the resulting frame
    cv2.imshow('frame', frame)
      
    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
