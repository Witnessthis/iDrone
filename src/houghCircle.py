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
from math import sqrt, acos, pi

def analyzeLines(image):
    img = image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = gray
    edges = cv2.GaussianBlur(edges, (15, 15), 0)
    edges = cv2.medianBlur(edges, 5)
    # edges = cv2.Canny(edges,50,150,apertureSize = 3)
    edges = cv2.Canny(edges, 50, 200, apertureSize=3)
    #cv2.imwrite('canny_hp.jpg', edges)
    # minLineLength = 100
    # maxLineGap = 10
    minLineLength = 80
    maxLineGap = 30
    votes = 10
    # lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength,maxLineGap)
    lines = cv2.HoughLinesP(edges, 1, pi / 180, votes, minLineLength, maxLineGap)
    # print("Line length: " + str(len(lines)))
    if lines is not None:
        for x in range(0, len(lines)):
            for x1, y1, x2, y2 in lines[x]:
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # print(lines[x])

    #cv2.imwrite('houghlines5.jpg', img)
    cv2.imshow("Tobias window: ",img)
    cv2.waitKey(1)

    def isInsideBorder(line, borders):
        xm = (line[0] + line[2]) / 2
        ym = (line[1] + line[3]) / 2
        xbl = borders[0]
        ybl = borders[1]
        xbu = borders[2]
        ybu = borders[3]

        if (xm > xbl and xm < xbu and ym > ybl and ym < ybu):
            # print("isInsideBorder")
            return True
        else:
            return False

    def isPerpendicular(line1, line2, accuracy):
        xa = line1[0]
        ya = line1[1]
        xb = line1[2]
        yb = line1[3]
        xc = line2[0]
        yc = line2[1]
        xd = line2[2]
        yd = line2[3]
        ab_x = xb - xa
        cd_x = xd - xc
        ab_y = yb - ya
        cd_y = yd - yc
        ab_mag = sqrt(ab_x * ab_x + ab_y * ab_y)
        cd_mag = sqrt(cd_x * cd_x + cd_y * cd_y)
        angle = acos((ab_x * cd_x + ab_y * cd_y) / (ab_mag * cd_mag))
        deg = angle * 180.0 / pi

        if (90 - accuracy < deg and deg < 90 + accuracy):
            return True
        else:
            return False

    xc = 440
    yc = 192
    r = 73
    accuracy = 20
    # lines = [[5,4,5,6],[4,3,6,3]]
    # lines  = [[57,510,363,510],[247,291,241,132]]
    # print("test")
    # framefactor
    ff = r * 2
    bordersCircle = [xc - r, yc - r, xc + r, yc + r]
    bordersFrame = [xc - ff, yc - ff, xc + ff, yc + ff]
    print("bordersCircle: " + str(bordersCircle))
    print("bordersFrame " + str(bordersFrame))
    # cout << isInsideBorder(lines[0],bordersCirle) << endl
    # cout << isInsideBorder(lines[1],bordersFrame) << endl
    # cout << isPerpendicular(lines, accuracy)
    linesInFrame = []
    if lines is not None:
        for i in range(0, len(lines)):
            # print(lines[0][0])
            # print(i)
            if (isInsideBorder(lines[i][0], bordersFrame)):
                linesInFrame.append(lines[i][0])
                # print(lines[i])

    if linesInFrame is not None:
        for i in range(0, len(linesInFrame)):
            if (isInsideBorder(linesInFrame[i], bordersCircle)):
                print(linesInFrame[i])
                for j in range(0, len(linesInFrame)):
                    if (not (isInsideBorder(linesInFrame[j], bordersCircle)) and isPerpendicular(linesInFrame[i],
                                                                                                 linesInFrame[j],
                                                                                                 accuracy) and i != j):
                        print("Great Success!")
                        # print(lines[i])
                        # return True;
def callback(image):
    # instantiate cvbridge and convert raw feed to cv image
    br = CvBridge()
    processImage = br.imgmsg_to_cv2(image, "bgr8")

    #analyzeLines(processImage)

    # convert image to grayscale for processing
    grayscale_image = cv2.cvtColor(processImage, cv2.COLOR_BGR2GRAY)

    # apply blurs to remove noise, experimenting with gaussian and median blur currently
    grayscale_image = cv2.GaussianBlur(grayscale_image, (5, 5), 0);
    grayscale_image = cv2.medianBlur(grayscale_image, 5)

    # using adaptive gaussian threshold to detect sharp edges in current frame
    grayscale_image = cv2.adaptiveThreshold(grayscale_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                            cv2.THRESH_BINARY,
                                            11, 3.5)

    # instantiate matrix for image processing
    kernel = np.ones((2.6, 2.7), np.uint8)

    # erode the grayscale image
    grayscale_image = cv2.erode(grayscale_image, kernel, iterations=1)

    # dilate the grayscale image
    grayscale_image = cv2.dilate(grayscale_image, kernel, iterations=1)

    # detect if there are circles in the processed image
    circles_detect = cv2.HoughCircles(grayscale_image, cv2.HOUGH_GRADIENT, 0.5, 246, param1=330, param2=63,
                                      minRadius=0, maxRadius=180)

    # calculate center of the image, save height and width coordinates
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

            # publish the coordinates of the circle
            pub = rospy.Publisher('circlecoordinate', afAdjust, queue_size=10)

            # setup the message type to publish and assign values
            coordinate_msg = afAdjust()
            coordinate_msg.c_x = float(x)
            coordinate_msg.c_y = float(y)
            coordinate_msg.imgc_x = float(width)
            coordinate_msg.imgc_y = float(height)
            coordinate_msg.match = 2

            # log and publish the message
            rospy.loginfo(coordinate_msg)
            pub.publish(coordinate_msg)
        #cv2.imshow('Output Image', processImage)
        #cv2.imshow('Processed grayscale Image', grayscale_image)
        #cv2.waitKey(0)

    # display the processed image and the output image for relation
    else:
        # the controller demands data even when no circle is found, therefor this is introduced
        pub = rospy.Publisher('circlecoordinate', afAdjust, queue_size=10)

        # assign values to the afAdjust message
        coordinate_msg = afAdjust()
        coordinate_msg.c_x = float(0)
        coordinate_msg.c_y = float(0)
        coordinate_msg.imgc_x = float(width)
        coordinate_msg.imgc_y = float(height)
        coordinate_msg.match = 0

        # log and publish the message
        rospy.loginfo(coordinate_msg)
        pub.publish(coordinate_msg)

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