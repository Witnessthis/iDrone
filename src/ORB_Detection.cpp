//
// Created by praem on 6/7/16.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <iDrone/afAdjust.h>
#include <math.h>


using namespace cv;
using namespace std;
//using namespace enc;

void selectiveImageAnalysisCallback(const sensor_msgs::ImageConstPtr& msg);

int message;

ros::Publisher chatter_pub;
std::vector <cv::Point2f> scene_corners(4);

Point2f finalPoint;
Point2f centerPoint;

float compare_max_value = 20;
float compare_min_value = -20;
float compare_max_value2 = 4;
float compare_min_value2 = -4;
Mat templateImage;

// TODO: Change the string in the loadPicture() method, to the path to your image location

/*
 * ROS messages
 * 1: GOOD_MATCH
 * 2: BAD_MATCH
 * 3: NO_MATCH
 *
 * format
 * type pointx pointy centerx centery
 *
 * to run this program, use rosrun, and consider adding some of the arguments on run,
 * if you wan't to run an image feed from other than the AR drone driver
 * arguments accepted:
 * run
 * debug
 * webcam
 *
 */

Mat computeMatch(Mat image1, std::vector <cv::DMatch> good_matches) {

    float corner_x_max = scene_corners[0].x + scene_corners[1].x + scene_corners[2].x + scene_corners[3].x;
    float corner_y_max = scene_corners[0].y + scene_corners[1].y + scene_corners[2].y + scene_corners[3].y;


    float corner_x_average, corner_y_average;
    finalPoint = cv::Point2f(0,0);
    std::vector <cv::Point2f> goodMatches_corners(4);
    corner_x_average = (corner_x_max / 4);
    corner_y_average = (corner_y_max / 4);

    cout << "coordinates: \n";
    cout << "Point 0: "<< scene_corners[0].x << ", " << scene_corners[0].y << "\n";
    cout << "Point 1: "<< scene_corners[1].x << ", " << scene_corners[1].y << "\n";
    cout << "Point 2: "<< scene_corners[2].x << ", " << scene_corners[2].y << "\n";
    cout << "Point 3: "<< scene_corners[3].x << ", " << scene_corners[3].y << "\n";

    // rectangle detection
    std::vector <cv::Point2f> sortedCorners(4);
    std::vector <cv::Point2f> sortedSides(4);
    std::vector <float> angles(4);
    std::vector <float> lengths(4);
    int sortingCounter = 0;


    // sort the corners in this pattern
    /*
     *      1------2
     *      |      |
     *      |      |
     *      |      |
     *      0------3
     */

    // find corner 0
    for (int i = 0; i<4; i++) {
        if (scene_corners[i].x < corner_x_average && scene_corners[i].y < corner_y_average) {
            sortedCorners[0] = scene_corners[i];
            sortingCounter++;
        }
    }
    // find corner 1
    for (int i = 0; i<4; i++) {
        if (scene_corners[i].x < corner_x_average && scene_corners[i].y > corner_y_average) {
            sortedCorners[1] = scene_corners[i];
            sortingCounter++;
        }
    }
    // find corner 2
    for (int i = 0; i<4; i++) {
        if (scene_corners[i].x > corner_x_average && scene_corners[i].y > corner_y_average) {
            sortedCorners[2] = scene_corners[i];
            sortingCounter++;
        }
    }
    //find corner 3
    for (int i = 0; i<4; i++) {
        if (scene_corners[i].x > corner_x_average && scene_corners[i].y < corner_y_average) {
            sortedCorners[3] = scene_corners[i];
            sortingCounter++;
        }
    }

    // detect wether the sorted corners forms a rectangle or not
    // finds the side vectors of the rectangle in this pattern
    /*          1
     *      1------2
     *      |      |
     *     0|      |2
     *      |      |
     *      0------3
     *          3
     */

    if (sortingCounter==4) {

        // finds the side vectors of the rectangle
        sortedSides[0]=Point2f((-1)*(sortedCorners[1].x-sortedCorners[0].x), (-1)*(sortedCorners[1].y-sortedCorners[0].y));
        sortedSides[1]=Point2f((-1)*(sortedCorners[2].x-sortedCorners[1].x), (-1)*(sortedCorners[2].y-sortedCorners[1].y));
        sortedSides[2]=Point2f((-1)*(sortedCorners[3].x-sortedCorners[2].x), (-1)*(sortedCorners[3].y-sortedCorners[2].y));
        sortedSides[3]=Point2f((-1)*(sortedCorners[0].x-sortedCorners[3].x), (-1)*(sortedCorners[0].y-sortedCorners[3].y));

        cout << "sorted sides: \n";
        cout << "Point 0: "<< sortedSides[0].x << ", " << sortedSides[0].y << "\n";
        cout << "Point 1: "<< sortedSides[1].x << ", " << sortedSides[1].y << "\n";
        cout << "Point 2: "<< sortedSides[2].x << ", " << sortedSides[2].y << "\n";
        cout << "Point 3: "<< sortedSides[3].x << ", " << sortedSides[3].y << "\n";

        // find the angles of the corners in the rectangle
        int dot0 = sortedSides[0].x * sortedSides[1].x + sortedSides[0].y * sortedSides[1].y;
        int det0 = sortedSides[0].x * sortedSides[1].y - sortedSides[0].y * sortedSides[1].x;
        angles[0] = atan2(det0, dot0);
        angles[0] = angles[0] * 180 / 3.14;

        int dot1 = sortedSides[1].x * sortedSides[2].x + sortedSides[1].y * sortedSides[2].y;
        int det1 = sortedSides[1].x * sortedSides[2].y - sortedSides[1].y * sortedSides[2].x;
        angles[1] = atan2(det1, dot1);
        angles[1] = angles[1] * 180 / 3.14;

        int dot2 = sortedSides[2].x * sortedSides[3].x + sortedSides[2].y * sortedSides[3].y;
        int det2 = sortedSides[2].x * sortedSides[3].y - sortedSides[2].y * sortedSides[3].x;
        angles[2] = atan2(det2, dot2);
        angles[2] = angles[2] * 180 / 3.14;

        int dot3 = sortedSides[3].x * sortedSides[0].x + sortedSides[3].y * sortedSides[0].y;
        int det3 = sortedSides[3].x * sortedSides[0].y - sortedSides[3].y * sortedSides[0].x;
        angles[3] = atan2(det3, dot3);
        angles[3] = angles[3] * 180 / 3.14;

        String angle0 = static_cast<ostringstream*>( &(ostringstream() << angles[0]) )->str();
        String angle1 = static_cast<ostringstream*>( &(ostringstream() << angles[1]) )->str();
        String angle2 = static_cast<ostringstream*>( &(ostringstream() << angles[2]) )->str();
        String angle3 = static_cast<ostringstream*>( &(ostringstream() << angles[3]) )->str();

        // finds the length of all side vectors
        lengths[0]=sqrt((sortedSides[0].x*sortedSides[0].x)+(sortedSides[0].y*sortedSides[0].y));
        lengths[1]=sqrt((sortedSides[1].x*sortedSides[1].x)+(sortedSides[1].y*sortedSides[1].y));
        lengths[2]=sqrt((sortedSides[2].x*sortedSides[2].x)+(sortedSides[2].y*sortedSides[2].y));
        lengths[3]=sqrt((sortedSides[3].x*sortedSides[3].x)+(sortedSides[3].y*sortedSides[3].y));

        String length0 = static_cast<ostringstream*>( &(ostringstream() << lengths[0]) )->str();
        String length1 = static_cast<ostringstream*>( &(ostringstream() << lengths[1]) )->str();
        String length2 = static_cast<ostringstream*>( &(ostringstream() << lengths[2]) )->str();
        String length3 = static_cast<ostringstream*>( &(ostringstream() << lengths[3]) )->str();

        cout << "Rectangle debug! \n";
        cout << "angle 0: " << angle0 << "\n";
        cout << "angle 1: " << angle1 << "\n";
        cout << "angle 2: " << angle2 << "\n";
        cout << "angle 3: " << angle3 << "\n";

        cout << "length 0: " << length0 << "\n";
        cout << "length 1: " << length1 << "\n";
        cout << "length 3: " << length2 << "\n";
        cout << "length 4: " << length3 << "\n";

        // detects if all angles are between 45 and 55 degrees
        // and wether all sides are bigger than the desired tresh
        int angleCounter = 0;
        int lengthCounter=0;
        for (int i = 0; i<4; i++) {
            // Test if angles are within the treshhold (negative angles are taken into account aswell)
            if (angles[i] >= 80.0 && angles[i] <= 100.0) angleCounter++;
            else if (angles[i] <= -80.0 && angles[i] >= -100.0) angleCounter++;
            // check if the length are within the treshhold
            if (lengths[i]>50) lengthCounter++;
        }

        String angleCounter0 = static_cast<ostringstream*>( &(ostringstream() << angleCounter) )->str();
        String lengthCounter0 = static_cast<ostringstream*>( &(ostringstream() << lengthCounter) )->str();

        cout << angleCounter0 << "\n";
        cout << lengthCounter0 << "\n";


        if (angleCounter==4 && lengthCounter==4) {
            // PERFECT MATCH
            cout << "RECTANGLE ACCEPTED \n";
            message = 3;
        } else {
            int lengthDenierCounter = 0;
            for (int i = 0; i<4; i++) {
                if (lengths[i]<100.0) lengthDenierCounter++;
            }

            if (lengthDenierCounter == 4) {
                // GOOD MATCH
                message = 2;
            } else {
                // BAD MATCH
                message = 1;
            }

        }
    } else {
        // bad match
        message = 1;
    }

    if (message==3 || message==2) {
        if (message==3) {
            // Draw lines
            cv::line(image1,
                     scene_corners[0],
                     scene_corners[1],
                     cv::Scalar(0, 255, 0), 4);
            cv::line(image1,
                     scene_corners[1],
                     scene_corners[2],
                     cv::Scalar(0, 255, 0), 4);
            cv::line(image1,
                     scene_corners[2],
                     scene_corners[3],
                     cv::Scalar(0, 255, 0), 4);
            cv::line(image1,
                     scene_corners[3],
                     scene_corners[0],
                     cv::Scalar(0, 255, 0), 4);
        }
        // Calculate center of found point (average point)
        finalPoint = finalPoint + scene_corners[0];
        finalPoint = finalPoint + scene_corners[1];
        finalPoint = finalPoint + scene_corners[2];
        finalPoint = finalPoint + scene_corners[3];
        finalPoint = finalPoint / 4;
        circle(image1, finalPoint, 20, cv::Scalar(255, 255, 0), 5, 8, 0);
        circle(image1, finalPoint, 3, cv::Scalar(255, 255, 0), 5, 8, 0);
        cout << finalPoint.x << ", " << finalPoint.y << "\n";


    } else {
        finalPoint = Point2f(0,0);
    }
    return image1;
}

Mat ORBTemplateMatch(Mat image1, Mat image2) {

    std::vector<cv::KeyPoint> keypoints_templ, keypoints_frame;
    Mat descriptors_object, descriptors_scene;
    Ptr<DescriptorExtractor> descriptorExtractor = ORB::create();
    Ptr<FeatureDetector> detector = ORB::create();

    detector->detect(image1, keypoints_templ);
    detector->detect(image2, keypoints_frame);

    descriptorExtractor->compute(image1, keypoints_templ, descriptors_object);
    descriptorExtractor->compute(image2, keypoints_frame, descriptors_scene);

    cv::drawKeypoints(image2, keypoints_frame, image2, cv::Scalar(0, 255, 255));
    cv::drawKeypoints(image1, keypoints_frame, image1, cv::Scalar(0, 255, 255));


    // FLANN matcher
    cv::BFMatcher matcher;
    std::vector<cv::DMatch> matches;
    cv::Mat img_matches;

    if (descriptors_object.data && descriptors_scene.data) {
        matcher.match(descriptors_object, descriptors_scene, matches);

        double max_dist = 500;
        double min_dist = 0;

        std::vector<cv::DMatch> good_matches;

        for (int i = 0; i < descriptors_object.rows; i++) {
            if (matches[i].distance < (max_dist / 1.6)
                && matches[i].distance > min_dist) { good_matches.push_back(matches[i]); }
        }

        cv::drawMatches(image1, keypoints_templ, image2, keypoints_frame, \
                good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                        std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);


        std::vector<cv::Point2f> obj;
        std::vector<cv::Point2f> scene;

        for (size_t i = 0; i < good_matches.size(); i++) {
            //-- get the keypoints from the good matches
            obj.push_back(keypoints_templ[good_matches[i].queryIdx].pt);
            scene.push_back(keypoints_frame[good_matches[i].trainIdx].pt);
        }
        cout << "found " << good_matches.size() << " matches \n";
        if (!obj.empty() && !scene.empty() && good_matches.size() >= 40) {
            cv::Mat H = cv::findHomography(obj, scene, cv::RANSAC);

            //-- get the corners from the object to be detected
            std::vector<cv::Point2f> obj_corners(4);
            obj_corners[0] = cv::Point(0, 0);
            obj_corners[1] = cv::Point(image1.cols, 0);
            obj_corners[2] = cv::Point(image1.cols, image1.rows);
            obj_corners[3] = cv::Point(0, image1.rows);

            cv::perspectiveTransform(obj_corners, scene_corners, H);

            // show matches
            //if (img_matches.data) imshow("1", img_matches);

            if (image2.data) {
                return computeMatch(image2, good_matches);
            }
            message = 0;
            return image2;
        } else {
            message = 0;
            return image2;
        }
    }
    message = 0;
    return image2;
}


Mat loadPicture(String imageToLoad){
    Mat picture;
    // TODO: Change this string to the path to your string location
    if (imageToLoad=="default") {
        picture = imread("/home/praem/catkin_ws/src/iDrone/src/Template.png");
    } else {
        picture = imread(imageToLoad);
    }
    return picture;
}

Mat loadFrame(int framenumber) {
    String convertion;

    String str = static_cast<ostringstream*>( &(ostringstream() << framenumber) )->str();

    if (framenumber<10) convertion="000";
    else if (framenumber<100) convertion="00";
    else if (framenumber<1000) convertion="0";
    else convertion = "";

    Mat picture;
    String picturepath = "/home/praem/catkin_ws/devel/lib/Opencv_test2/Flyvetest1/frame"
                         +convertion+str+".jpg";
    cout << picturepath << "\n";
    picture = imread(picturepath);
    return picture;
}
void runImageLoop(Mat imageToAnalysis) {
    if (!imageToAnalysis.data) {
        //cout << "Could not run simulation \n" << std::endl;
        return;
    }
    message=0;
    centerPoint = cv::Point2f(imageToAnalysis.cols / 2, imageToAnalysis.rows / 2);
    std::vector <cv::Point2f> crosshairPoints(4);
    crosshairPoints[0] = cv::Point2f(imageToAnalysis.cols / 2, 0);
    crosshairPoints[1] = cv::Point2f(imageToAnalysis.cols / 2, imageToAnalysis.rows);
    crosshairPoints[2] = cv::Point2f(0, imageToAnalysis.rows / 2);
    crosshairPoints[3] = cv::Point2f(imageToAnalysis.cols, imageToAnalysis.rows / 2);

    imageToAnalysis.convertTo(imageToAnalysis, CV_8U);
    cvtColor(imageToAnalysis, imageToAnalysis, CV_BGR2GRAY);
    blur(imageToAnalysis, imageToAnalysis, Size(3, 3));
    Mat resultImage;
    try {
        resultImage = ORBTemplateMatch(templateImage, imageToAnalysis);
    } catch (Exception e) {
        return;
    }
    // ROS message
    iDrone::afAdjust msg;
    //String str_result = static_cast<ostringstream*>( &(ostringstream() << message) )->str();
    //cout << str_result <<"\n";
    msg.match = message;
    msg.c_x = finalPoint.x;
    msg.c_y = finalPoint.y;
    msg.imgc_x = centerPoint.x;
    msg.imgc_y = centerPoint.y;
    ROS_INFO("%i %f %f %f %f", message, finalPoint.x, finalPoint.y, centerPoint.x, centerPoint.y);
    chatter_pub.publish(msg);

    // draw picture center and crosshair
    circle(resultImage, centerPoint, 3, cv::Scalar(255, 0, 255), 5, 8, 0);
    cv::line(resultImage, crosshairPoints[0], crosshairPoints[1], cv::Scalar(255, 0, 255), 3);
    cv::line(resultImage, crosshairPoints[2], crosshairPoints[3], cv::Scalar(255, 0, 255), 3);

    cv::imshow("result", resultImage);
    waitKey(3);
}

void selectiveImageAnalysisCallback(const sensor_msgs::ImageConstPtr& msg){
    Mat inputImage;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        inputImage = cv_ptr->image;
        runImageLoop(inputImage);
    } catch(cv_bridge::Exception& e) {
        ROS_ERROR("could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

}

int main(int argc, char* argv[])
{
    // Init ros
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    namedWindow("result");
    startWindowThread();

    image_transport::ImageTransport it(n);

    chatter_pub = n.advertise<iDrone::afAdjust>("ORB_Detection", 1000);
    String argc_str = static_cast<ostringstream*>( &(ostringstream() << argc) )->str();

    templateImage = loadPicture("default");
    if (!templateImage.data)
    {
        cout << "Could not open or find template image, try to change the path in the source code \n" << std::endl;
        return -1;
    }
    templateImage.convertTo(templateImage, CV_8U);
    cvtColor(templateImage, templateImage, CV_BGR2GRAY);
    blur(templateImage, templateImage, Size(3, 3));

    if (argc < 2) {
        // Running without arguments
        // Standard run procedure

        ros::Subscriber bottomImageRaw = n.subscribe("ardrone/bottom/image_raw", 1, selectiveImageAnalysisCallback);
        cout << "Subscribed to ardrone bottom camera\n";
        ros::spin();


    } else if (argc == 2){
        // Running with arguments
        String command = argv[1];
        cout << "running with command: " << command << "\n";

        if (command == "run") {
            cout << "Started subscribing on drone camera \n";
            ros::Subscriber bottomImageRaw = n.subscribe("ardrone/bottom/image_raw", 1, selectiveImageAnalysisCallback);
            ros::spin();
        }

        else if (command == "debug") {
            cout << "running debug mode \n";
            int counter = 0;
            Mat loadedFrameImage;
            while (1) {
                loadedFrameImage = loadFrame(counter);
                runImageLoop(loadedFrameImage);
                counter++;
                ros::spinOnce();
                waitKey(0);
            }
        }

        else if (command == "webcam"){
            cout << "running orbdetection with webcam \n";
            Mat frame;
            VideoCapture cap(0); // open the default camera
            if(!cap.isOpened())  // check if we succeeded
                return -1;
            while(1) {
                cap >> frame;
                runImageLoop(frame);
                ros::spinOnce();
                waitKey(0);
            }
        }

        else {
            Mat inputImage = loadPicture(command);
            runImageLoop(inputImage);
            waitKey(0);
        }
    }
    cv::destroyWindow("result");
    return 0;
}