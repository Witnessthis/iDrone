#!/usr/bin/env python
import roslib
roslib.load_manifest('iDrone')
import sys
import rospy
import cv2
import numpy as np
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from iDrone.msg import afAdjust


# list of current videos in folder, comment current and uncomment another for testing
#capture_device = cv2.VideoCapture('../videos/hallo.avi')
#capture_device = cv2.VideoCapture('../videos/hej.avi')
#capture_device = cv2.VideoCapture('../videos/whatever.avi')
#capture_device = cv2.VideoCapture('../videos/flight_1.mp4')
#capture_device = cv2.VideoCapture('../videos/VideoTest1.avi')
#capture_device = cv2.VideoCapture('../videos/flight.mp4')

# comment all videos above and uncomment this for webcam video
#capture_device = cv2.VideoCapture(0)

# Optional setting for width and height, uncomment to activate
#capture_device.set(3,640)
#capture_device.set(4,360)

def callback(image):
    # instantiate cvbridge and convert raw feed to cv image
    br = CvBridge()
    processImage = br.imgmsg_to_cv2(image, "bgr8")

    # convert image to grayscale for processing
    grayscale_image = cv2.cvtColor(processImage, cv2.COLOR_BGR2GRAY)

    # apply blurs to remove noice, experimenting with gaussian and median blur currently
    grayscale_image = cv2.GaussianBlur(grayscale_image, (5, 5), 0);
    grayscale_image = cv2.medianBlur(grayscale_image, 5)

    # using adaptive gaussian threshold to detect sharp edges in current frame
    grayscale_image = cv2.adaptiveThreshold(grayscale_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                            cv2.THRESH_BINARY,
                                            11, 3.5)

    kernel = np.ones((2.6, 2.7), np.uint8)

    # erode the grayscale image
    grayscale_image = cv2.erode(grayscale_image, kernel, iterations=1)

    # dilate the grayscale image
    grayscale_image = cv2.dilate(grayscale_image, kernel, iterations=1)

    # detect if there are circles in the processed image
    circles_detect = cv2.HoughCircles(grayscale_image, cv2.HOUGH_GRADIENT, 0.5, 246, param1=330, param2=63,
                                      minRadius=0, maxRadius=180)

    # calculate center of the image and draw a circle to show it
    height = np.size(processImage, 0) / 2
    width = np.size(processImage, 1) / 2

    # show center of image with a small purple'ish circle
    cv2.circle(processImage, (width, height), 5, (105, 0, 150), 3)

    # make sure there are circles found in the image
    if circles_detect is not None:
        # convert the (x, y) coordinates and radius of the circles found to integers
        circles_detect = np.round(circles_detect[0, :]).astype("int")

        # iterate through the circles detected, their x, y coordinates and radius
        for (x, y, r) in circles_detect:
            # draw the circle and center of the circle in the output image
            cv2.circle(processImage, (x, y), r, (0, 200, 50), 4)
            cv2.rectangle(processImage, (x - 5, y - 5), (x + 5, y + 5), (0, 150, 250), -1)

            # delay the image showing to be able to notice the drawings on the output image
            #            time.sleep(0.5)

            # publish the coordinates of the circle
            pub = rospy.Publisher('circlecoordinate', afAdjust, queue_size=10)

            # setup a delay for processing
            #rate = rospy.Rate(10)  # 10hz

            # setup the message type to publish and assign values
            coordinate_msg = afAdjust()
            coordinate_msg.c_x = float(width - x)
            coordinate_msg.c_y = float(height - y)
            coordinate_msg.imgc_x = float(width / 2)
            coordinate_msg.imgc_y = float(height / 2)
            coordinate_msg.match = 2

            # print diameter of the circle to terminal
            print "Diameter of circle: " + str(r * 2)

            # log and publish the message
            rospy.loginfo(coordinate_msg)
            pub.publish(coordinate_msg)

            # sleep rate ms
            #rate.sleep()

    # display the processed image and the output image for relation
    cv2.imshow('Output Image', processImage)
    cv2.imshow('Processed grayscale Image', grayscale_image)

    # keep frame alive
    cv2.waitKey(1)

def main(args):
    # initialize node
    rospy.init_node('houghCircle', anonymous=True)

    # subscribe on bottom_ardrone_camera
    rospy.Subscriber("/ardrone/bottom/image_raw", Image, callback, queue_size = 1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)