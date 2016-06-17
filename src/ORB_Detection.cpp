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

// TODO: Change the string in the loadPicture() method, to the path to your string location

/*
 * ROS messages
 * 1: GOOD_MATCH
 * 2: BAD_MATCH
 * 3: NO_MATCH
 *
 * format
 * 1: 1 pointx pointy centerx centery
 * 2: 2
 * 3: 3
 */

Mat computeMatch(Mat image1) {
    int status_corner0_x=1, status_corner0_y=1, status_corner1_x=1, status_corner1_y=1,
            status_corner2_x=1, status_corner2_y=1, status_corner3_x=1, status_corner3_y=1;
    bool accepted0 = false, accepted1 = false, accepted2 = false, accepted3 = false;
    float corner_x_max = scene_corners[0].x + scene_corners[1].x + scene_corners[2].x + scene_corners[3].x;
    float corner_y_max = scene_corners[0].y + scene_corners[1].y + scene_corners[2].y + scene_corners[3].y;
    float cornerAcceptedAverage_x = 0, cornerAcceptedAverage_y = 0;
    int acceptedCounter = 0;

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


    // Filter bad results
    if (corner_x_average - scene_corners[0].x > compare_max_value || corner_x_average - scene_corners[0].x < compare_min_value) {
        status_corner0_x = 0;
    } else {
        cornerAcceptedAverage_x += scene_corners[0].x;
    }
    if (corner_y_average - scene_corners[0].y > compare_max_value || corner_y_average - scene_corners[0].y < compare_min_value) {
        status_corner0_y = 0;
    } else {
        cornerAcceptedAverage_y += scene_corners[0].y;
    }
    if (corner_x_average - scene_corners[1].x > compare_max_value || corner_x_average - scene_corners[1].x < compare_min_value) {
        status_corner1_x = 0;
    } else {
        cornerAcceptedAverage_x += scene_corners[1].x;
    }
    if (corner_y_average - scene_corners[1].y > compare_max_value || corner_y_average - scene_corners[1].y < compare_min_value) {
        status_corner1_y = 0;
    } else {
        cornerAcceptedAverage_y += scene_corners[1].y;
    }
    if (corner_x_average - scene_corners[2].x > compare_max_value || corner_x_average - scene_corners[2].x < compare_min_value) {
        status_corner2_x = 0;
    } else {
        cornerAcceptedAverage_x += scene_corners[2].x;
    }
    if (corner_y_average - scene_corners[2].y > compare_max_value || corner_y_average - scene_corners[2].y < compare_min_value) {
        status_corner2_y = 0;
    } else {
        cornerAcceptedAverage_y += scene_corners[2].y;
    }
    if (corner_x_average - scene_corners[3].x > compare_max_value || corner_x_average - scene_corners[3].x < compare_min_value) {
        status_corner3_x = 0;
    } else {
        cornerAcceptedAverage_x += scene_corners[3].x;
    }
    if (corner_y_average - scene_corners[3].y > compare_max_value || corner_y_average - scene_corners[3].y < compare_min_value) {
        status_corner3_y = 0;
    } else {
        cornerAcceptedAverage_y += scene_corners[3].y;
    }

    cornerAcceptedAverage_x = cornerAcceptedAverage_x / (status_corner0_x + status_corner1_x +
                                                         status_corner2_x + status_corner3_x);

    cornerAcceptedAverage_y = cornerAcceptedAverage_y / (status_corner0_y + status_corner1_y +
                                                         status_corner2_y + status_corner3_y);

    if (cornerAcceptedAverage_x - scene_corners[0].x > compare_max_value ||
            cornerAcceptedAverage_x - scene_corners[0].x < compare_min_value) {
        status_corner0_x = 0;
    } else {
        status_corner0_x = 1;
    }
    if (cornerAcceptedAverage_y - scene_corners[0].y > compare_max_value ||
            cornerAcceptedAverage_y - scene_corners[0].y < compare_min_value) {
        status_corner0_y = 0;
    } else {
        status_corner0_y = 1;
    }
    if (cornerAcceptedAverage_x - scene_corners[1].x > compare_max_value ||
            cornerAcceptedAverage_x - scene_corners[1].x < compare_min_value) {
        status_corner1_x = 0;
    } else {
        status_corner1_x = 1;
    }
    if (cornerAcceptedAverage_y - scene_corners[1].y > compare_max_value ||
            cornerAcceptedAverage_y - scene_corners[1].y < compare_min_value) {
        status_corner1_y = 0;
    } else {
        status_corner1_y = 1;
    }
    if (cornerAcceptedAverage_x - scene_corners[2].x > compare_max_value ||
            cornerAcceptedAverage_x - scene_corners[2].x < compare_min_value) {
        status_corner2_x = 0;
    } else {
        status_corner2_x = 1;
    }
    if (cornerAcceptedAverage_y - scene_corners[2].y > compare_max_value ||
            cornerAcceptedAverage_y - scene_corners[2].y < compare_min_value) {
        status_corner2_y = 0;
    } else {
        status_corner2_y = 1;
    }
    if (cornerAcceptedAverage_x - scene_corners[3].x > compare_max_value ||
            cornerAcceptedAverage_x - scene_corners[3].x < compare_min_value) {
        status_corner3_x = 0;
    } else {
        status_corner3_x = 1;
    }
    if (cornerAcceptedAverage_y - scene_corners[3].y > compare_max_value ||
            cornerAcceptedAverage_y - scene_corners[3].y < compare_min_value) {
        status_corner3_y = 0;
    } else {
        status_corner3_y = 1;
    }


    if ((status_corner0_x + status_corner0_y + status_corner1_x + status_corner1_y +
            status_corner2_x + status_corner2_y + status_corner3_x + status_corner3_y) < 4) {
        message = 1;
        return image1;
    }

    cout << "accepted coordinates: \n";
    if (status_corner0_x+status_corner0_y==2) {
        goodMatches_corners[0]=scene_corners[0];
        cout << "Point 0: "<< goodMatches_corners[0].x << ", " << goodMatches_corners[0].y << "\n";
        circle(image1, scene_corners[0], 30, cv::Scalar(0,255,255), 10, 8, 0);
        accepted0 = true;
    } else {
        circle(image1, scene_corners[0], 30, cv::Scalar(255,0,0), 10, 8, 0);
    }
    if (status_corner1_x+status_corner1_y==2) {
        goodMatches_corners[1]=scene_corners[1];
        cout << "Point 1: "<< goodMatches_corners[1].x << ", " << goodMatches_corners[1].y << "\n";
        circle(image1, scene_corners[1], 30, cv::Scalar(0,255,255), 10, 8, 0);
        accepted1 = true;
    } else {
        circle(image1, scene_corners[1], 30, cv::Scalar(255,0,0), 10, 8, 0);
    }
    if (status_corner2_x+status_corner2_y==2) {
        goodMatches_corners[2]=scene_corners[2];
        cout << "Point 2: "<< goodMatches_corners[2].x << ", " << goodMatches_corners[2].y << "\n";
        circle(image1, scene_corners[2], 30, cv::Scalar(0,255,255), 10, 8, 0);
        accepted2 = true;
    } else {
        circle(image1, scene_corners[2], 30, cv::Scalar(255,0,0), 10, 8, 0);
    }
    if (status_corner3_x+status_corner3_y==2) {
        goodMatches_corners[3]=scene_corners[3];
        cout << "Point 3: "<< goodMatches_corners[3].x << ", " << goodMatches_corners[3].y << "\n";
        circle(image1, scene_corners[3], 30, cv::Scalar(0,255,255), 10, 8, 0);
        accepted3 = true;
    } else {
        circle(image1, scene_corners[3], 30, cv::Scalar(255,0,0), 10, 8, 0);
    }
    if (accepted0) {
        finalPoint = finalPoint + goodMatches_corners[0];
        acceptedCounter++;
    }
    if (accepted1) {
        finalPoint = finalPoint + goodMatches_corners[1];
        acceptedCounter++;
    }
    if (accepted2) {
        finalPoint = finalPoint + goodMatches_corners[2];
        acceptedCounter++;
    }
    if (accepted3) {
        finalPoint = finalPoint + goodMatches_corners[3];
        acceptedCounter++;
    }

    // Add posible rectangle detection
    if (acceptedCounter==4)  {
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

    if (acceptedCounter > 1) {
        cout << "airfield center: ";
        finalPoint = finalPoint / acceptedCounter;
        circle(image1, finalPoint, 20, cv::Scalar(255, 255, 0), 5, 8, 0);
        circle(image1, finalPoint, 3, cv::Scalar(255, 255, 0), 5, 8, 0);
        cout << finalPoint.x << ", " << finalPoint.y << "\n";
        message = 2;
    } else {
        finalPoint = Point2f(0,0);
        message = 1;
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

            if (image2.data) {
                return computeMatch(image2);
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
        picture = imread("/home/lime/catkin_ws/src/iDrone/src/Template.png");
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
    namedWindow("result", WINDOW_NORMAL);

    // Init ros
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
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

        else {
            Mat inputImage = loadPicture(command);
            runImageLoop(inputImage);
            waitKey(0);
        }
    }
    return 0;
}