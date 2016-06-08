
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ardrone_autonomy/Navdata.h>
#include <thread>
#include <mutex>

#include <chrono>

#include "FiniteStateMachine.h"

//====== Variables and objects ======
ardrone_autonomy::Navdata navdata;
bool run = 1;
std::mutex navLock;

FiniteStateMachine fsm;

//====== Function prototypes ======
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void navdataHandler(ardrone_autonomy::Navdata in_navdata);
void iDroneFSM();

int main(int argc, char **argv)
{

    ros::init(argc, argv, "iDroneNode");
    ros::NodeHandle nh;

    ros::Subscriber navdata_sub = nh.subscribe("ardrone/navdata", 1000, navdataHandler);
    cv::namedWindow("view");
    cv::startWindowThread();
    ros::Subscriber frontImageRaw_sub = nh.subscribe("ardrone/front/image_raw", 1000, imageCallback);

    std::thread FSMThread(iDroneFSM);

    ros::spin();
    run = 0;

    cv::destroyWindow("view");
    FSMThread.join();
}


//====== Function implementations ======
void navdataHandler(ardrone_autonomy::Navdata in_navdata){
    std::cout << "navData" << std::endl;

    navLock.lock();
    navdata = in_navdata;
    navLock.unlock();

    fsm.update();
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

void iDroneFSM() {

    while (run) {

        fsm.act();

        //std::this_thread::sleep_for(std::chrono::seconds(1));

    }
}