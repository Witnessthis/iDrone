
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ardrone_autonomy/Navdata.h>
#include "RunNode.h"
//#include <pthread.h>

//====== Variables and objects ======
ardrone_autonomy::Navdata navdata;
RunNode run(&navdata);

//====== Function prototypes ======
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void navdataHandler(ardrone_autonomy::Navdata in_navdata);

int main(int argc, char **argv)
{
    int rc; // thread status
    pthread_t runThread;

    ros::init(argc, argv, "iDroneNode");
    ros::NodeHandle nh;

    ros::Subscriber navdata_sub = nh.subscribe("ardrone/navdata", 1000, navdataHandler);
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    ros::Subscriber frontImageRaw_sub = nh.subscribe("ardrone/front/image_raw", 1000, imageCallback);

    /*rc = pthread_create(&runThread, NULL, &run.iDroneFSM, (void *)&navdata);
    if (rc){
        std::cout << "Error:unable to create thread," << rc << std::endl;
        exit(-1);
    }*/

    ros::spin();
    cv::destroyWindow("view");
//    pthread_exit(&runThread);
}


//====== Function implementations ======
void navdataHandler(ardrone_autonomy::Navdata in_navdata){
    navdata = in_navdata;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}