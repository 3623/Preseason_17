import cv2
import sys,os,time
import threading
sys.path.insert(0,"../") 
import pyMjpgStreamer

def main():

    capture = cv2.VideoCapture(0)
    print('Camera stream opened')

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320); 
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240);
    #capture.set(cv2.cv.CV_CAP_PROP_SATURATION,0.2);

    streamer = pyMjpgStreamer.pyMjpgStreamer();
    
    while(1):
        try:
            status,image = capture.read()
            streamer.setImg(image,status)
        except KeyboardInterrupt:
            break;
        
    streamer.stop()
    print("Closing camera stream")
    cv2.destroyAllWindows()
    capture.release()   


#Actual main code starts here
if __name__ == "__main__":
    main()