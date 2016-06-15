#!/usr/bin/env python
import cv2
import numpy as np
import time
import rospy
from std_msgs.msg import String
from iDrone.msg import afAdjust


# list of current videos in folder, comment current and uncomment another for testing
capture_device = cv2.VideoCapture('../videos/hallo.avi')
#capture_device = cv2.VideoCapture('../videos/hej.avi')
#capture_device = cv2.VideoCapture('../videos/whatever.avi')
#capture_device = cv2.VideoCapture('../videos/flight_1.mp4')
#capture_device = cv2.VideoCapture('../videos/VideoTest1.avi')
#capture_device = cv2.VideoCapture('../videos/flight_1.mp4')

# comment all videos above and uncomment this for webcam video
#capture_device = cv2.VideoCapture(0)

# Optional setting for width and height, uncomment to activate
#capture_device.set(3,640)
#capture_device.set(4,360)

# Start capturing until video is finished or pressing 'q'
while(True):
    # Start capturing frame by frame
    ret, frame = capture_device.read()

    # load current image, copy it for outputting with circle and convert it to grayscale for processing
    output_image = frame.copy()
    grayscale_image = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # apply blurs to remove noice, experimenting with gaussian and median blur currently
    grayscale_image = cv2.GaussianBlur(grayscale_image,(5,5),0);
    grayscale_image = cv2.medianBlur(grayscale_image,5)

    # using adaptive gaussian threshold to detect sharp edges in current frame
    grayscale_image = cv2.adaptiveThreshold(grayscale_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 3.5)

    kernel = np.ones((2.6,2.7),np.uint8)

    # erode the grayscale image
    grayscale_image = cv2.erode(grayscale_image,kernel,iterations = 1)

    # dilate the grayscale image
    grayscale_image = cv2.dilate(grayscale_image,kernel,iterations = 1)

    # detect if there are circles in the processed image
    circles_detect = cv2.HoughCircles(grayscale_image, cv2.HOUGH_GRADIENT, 0.5, 246, param1 = 330, param2 = 63, minRadius = 0, maxRadius = 180)

    # calculate center of the image and draw a circle to show it
    height = np.size(output_image, 0) / 2
    width = np.size(output_image, 1) / 2
    cv2.circle(output_image, (width, height), 5, (105, 0, 150), 3)
    # make sure there are circles found in the image
    if circles_detect is not None:
        # convert the (x, y) coordinates and radius of the circles found to integers
        circles_detect = np.round(circles_detect[0, :]).astype("int")

        # iterate through the circles detected, their x, y coordinates and radius
        for (x,y,r) in circles_detect:
            # draw the circle and center of the circle in the output image
            cv2.circle(output_image, (x,y),r,(0, 200, 50),4)
            cv2.rectangle(output_image, (x-5,y-5),(x+5,y+5),(0,150,250),-1)

            # delay the image showing to be able to notice the drawings on the output image
#            time.sleep(0.5)

            # publish the coordinates of the circle
            pub = rospy.Publisher('circlecoordinate', afAdjust, queue_size=10)
            rospy.init_node('houghCircle', anonymous=True)
            rate = rospy.Rate(10)  # 10hz
            #while not rospy.is_shutdown():
#            coordinates_string = "\nX-middle: " + str(width) + "\nY-middle: " + str(height)
#            coordinates_string = coordinates_string + "\nX-coordinate: " + str(x / 2) + "\nY-coordinate: " +  str(y / 2) + "\nCircle Radius: " + str(r / 2) + "\n"
#            coordinates_string = coordinates_string + "\nOffset x-coordinate: " + str(width - x) + "\nOffset y-coordinate: " + str(height-y) + "\n\n"
            coordinate_msg = afAdjust()
            coordinate_msg.c_x = width - x
            coordinate_msg.c_y = height - y
            coordinate_msg.imgc_x = str(width/2)
            coordinate_msg.imgc_y = str(height/2)
            coordinate_msg.match = 2
            rospy.loginfo(coordinate_msg)
            pub.publish(coordinate_msg)
            rate.sleep()


            # print the found (x, y) coordinate and radius of the given circle

      #      print coordinates_string

    # display the processed image and the output image for relation
    cv2.imshow('Processed grayscale Image', grayscale_image)
    cv2.imshow('Output Image', output_image)

    # enable option to break while-loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#when while loop is finished, free resources
capture_device.release()
cv2.destroyAllWindows()
