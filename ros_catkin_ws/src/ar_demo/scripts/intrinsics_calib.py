#!/usr/bin/env python
import cv
import cv2
import rospy
import imutils
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import glob
import time
import pickle
import sys
import Queue


# TODO
# De distortie wordt gewoon elke keer berekend. Zet een stop op aantal beelden of zo.
# Feedback van de gevonden eindpunten is nog niet ok. Eventueel tonen met waitKey() die luistert naar k (keep) / d (drop)
# Creeer een control en status topic.

# schaalfactor stopconditie
stopScale = 10
stopImg = None
height = width = 0
nrImgs=0

goodImages=Queue.Queue()

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)



    
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
    
# images = glob.glob('/home/pieter/host/IntrinsicsRoutine/*.bmp') # Where the images can be found

class IntrinsicsCalibrationTool:


    def __init__(self, camera):
        cv2.namedWindow("Intrinsics Calibration Tool")  
        cv2.namedWindow("Stop image")
        
        
        # Chess board dimensions
        if(True): # A4 Bord
            self.size= 0.02518 # Size of checkerboard in meter
            self.patterncorners= (9, 6)
            self.iterations = 2 # Dilation
            print "Using A4 board with dimension: " + str(self.patterncorners)
        if(False): # A3 Bord
            self.size = 0.0375
            #size= 0.033875 # Size of checkerboard in meter
            # The number of corners are the inner corners
            # In principe rij,kolom
            self.patterncorners= (9, 6) # Pieter > Drive
            self.iterations = 4 # Dilation
            print "Using A3 board with dimension: " + str(self.patterncorners)
  
        self.objp = np.zeros((self.patterncorners[0]*self.patterncorners[1],3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.patterncorners[0],0:self.patterncorners[1]].T.reshape(-1,2)*self.size
        
        self.reset()
  
        self.image_bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber(
                "/camera/" + camera + "/stream", Image, self.imageCallback, queue_size = 1)
        self.camera_name = camera
        rospy.loginfo("Intrinsics calibration tool ready.")

	print "Press ENTER to accept the shown image."
	
	while True:
	  k = cv2.waitKey()
	  if(k==99): # Press 'c' to proceed to calibration.
	    break
	  elif(k==27):
	    pass

	  if(not goodImages.empty()):
    	    img = goodImages.get()
	    res = self.addImage(img)
	  else:
	    print "GoodImages empty"

	print 'Starting calibration'
	self.doCalibration()



    def reset(self):
        global stopImg
        global nrImgs
        stopImg = None
        nrImgs = 0
        

    def imageCallback(self, message):
        try:
          cv_image = self.image_bridge.imgmsg_to_cv2(message, "bgr8")
        except CvBridgeError as e:
          print(e)
          
        self.height, self.width = cv_image.shape[:2]
        
	if(self.testImage(cv_image)):
	  goodImages.put(cv_image)
	  print 'Images in queue: ' + str(goodImages.qsize())
        


    """Returns true if img gives valid chessboard
    """
    def testImage(self, img):
        global stopImg
        global nrImgs
        # Convert to grey
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        self.height, self.width = img.shape[:2]
            
    
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, self.patterncorners,corners=None, flags=cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_NORMALIZE_IMAGE | cv.CV_CALIB_CB_FILTER_QUADS)
    
	return ret




    def addImage(self, img):
        global stopImg
        global nrImgs
        # Convert to grey
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        self.shape = gray.shape
        self.height, self.width = img.shape[:2]
        
            
        # Initialise stop Img
        if stopImg is None:
            print "Image dimensions: " + str(self.width) + "-" + str(self.height)
            stopImg = np.zeros((self.height/stopScale,self.width/stopScale,1), np.uint8)
            
        if not(stopImg is None):
            sih, siw = stopImg.shape[:2]
            calibratieRand = 25 # Percentage
            randh = (self.height / stopScale) * calibratieRand/100 # Buitenste 10% beschouwen we als rand die we negeren
            randw = (self.width / stopScale) * calibratieRand/100
            
    
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, self.patterncorners,corners=None, flags=cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_NORMALIZE_IMAGE | cv.CV_CALIB_CB_FILTER_QUADS)
    
        # If found, add object points, image points (after refining them)
        if ret == True:
            #
            cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),criteria)
            
    
            # Draw and display the corners
            img_with_corners =img.copy()
            cv2.drawChessboardCorners(img_with_corners, self.patterncorners, corners,ret)
            
            # Show corners for visual inspection
            cv_image_reduced = imutils.resize(img_with_corners, width=self.width/4, height=self.height/4)
            cv2.imshow("Intrinsics Calibration Tool", cv_image_reduced)
            cv2.waitKey(20)
            
            k = cv2.waitKey()
            #print k
            if (k==27):    # Enter key to stop (27=Esc, 10=Enter)
                #print "Drop all"
		#with self.goodImages.mutex:
    		#	self.goodImages.queue.clear()
                #return None
		print "Not adding this image"
		return False
            elif (k==10):
		print "Adding this image corners"
		imgpoints.append(corners)

	        # Log corners in stopImg
	        pixels, _, _ = corners.shape # (70, 1, 2)
	        up = 35
	        for pixel in range(1, pixels):
	            cornerY =  int(corners[pixel][0][0] / stopScale) 
	            cornerX = int(corners[pixel][0][1] / stopScale) # Corner y

	            if (stopImg[cornerX,cornerY] + up) <  np.iinfo(np.uint8).max:
	                stopImg[cornerX,cornerY] =  stopImg[cornerX,cornerY] + up

                # Connect registered corners
                thresholdvalue = 14 # TODO Litteral
                ret,stopImgEval = cv2.threshold(stopImg,thresholdvalue,255,cv2.THRESH_BINARY)
                stopImgEval = cv2.dilate(stopImgEval, None, iterations=self.iterations)
                cv2.rectangle(stopImgEval, (randw-1, randh-1), (siw - randw + 1, sih - randh + 1), (127), 1)

                # Show stop condition state
                cv2.imshow("Stop image", stopImgEval)
                cv2.waitKey(20)

            	# Add corner points
            	nrImgs = nrImgs + 1
            	print "Valid calibration image " + str(nrImgs)
            	objpoints.append(self.objp)

		return True
	   

    def doCalibration(self):
	print "Calculating Calibration"
	#ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
	ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, self.shape[::-1],None,None)
	    
	print "Camera matrix:"
	print mtx
	print "Distortion coefficients:"
	print dist

	# Convert result to ROS CameraInfo message.
	K = [0.] * 9
	K[0] = mtx[0][0]
	K[2] = mtx[0][2]
	K[4] = mtx[1][1]
	K[5] = mtx[1][2]
	K[8] = 1.
	R = [0.] * 9
	R[0] = 1.
	R[4] = 1.
	R[8] = 1.
	P = [0.] * 12
	P[0] = mtx[0][0]
	P[2] = mtx[0][2]
	P[5] = mtx[1][1]
	P[6] = mtx[1][2]
	P[10] = 1.
	if len(dist[0]) == 4 or len(dist[0]) == 5:
	    distortion_model = 'plumb_bob'
	    D = [0.] * 5
	    for i, val in enumerate(dist[0]):
		D[i] = val
	elif len(dist[0]) == 8:
	    distortion_model = 'rational_polynomial'
	    D = [0.] * 8
	    for i, val in enumerate(dist[0]):
		D[i] = val
	else:
	    rospy.logerr("Unknown distortion model!")

	camera_info = CameraInfo(height = self.height, width = self.width,
		distortion_model = distortion_model, D = D,
		K = K, R = R, P = P,
		binning_x = 1, binning_y = 1)
	print camera_info
	# Save CameraInfo message to camera.
	try:
	    rospy.loginfo("Trying to save camera calibration...")
	    set_camera_info = rospy.ServiceProxy('/camera/' + self.camera_name + '/set_camera_info', SetCameraInfo)
	    response = set_camera_info(camera_info)
	    if response.success:
		rospy.loginfo("Camera calibration saved.")
	    else:
		rospy.logwarn("Camera calibration not saved: " + response.status_message)
	except rospy.ServiceException, e:
	    rospy.logerror("Service call failed: " + e)

	# Stats
	mean_error = 0
	for i in xrange(len(objpoints)):
	    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
	    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
	    mean_error += error
	    
	print "total error (should be well below 1): ", mean_error/len(objpoints)

            
 

## Start tool.
if __name__ == "__main__":
    rospy.init_node('intrinsics_calibration_tool_node')
    print sys.argv[1]
    intrinsics_calibration_tool = IntrinsicsCalibrationTool(sys.argv[1])
    rospy.spin()

