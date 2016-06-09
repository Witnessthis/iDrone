
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <mutex>

#include <chrono>

#include "FiniteStateMachine.h"
#include "model.h"
#include "ControlPanel.h"

#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"


//====== Variables and objects ======

model_s model;
bool run = 1;
std::mutex navLock;
std::mutex pubLock;

FiniteStateMachine fsm;

ros::Publisher takeoff_pub;
ros::Publisher land_pub;
ros::Publisher reset_pub;
ros::Publisher vel_pub;

//====== Function prototypes ======
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void navdataHandler(ardrone_autonomy::Navdata in_navdata);
void wallQRHandler(iDrone::qrAdjust msg);
void qrSpottedHandler(const std_msgs::String::ConstPtr& msg);

void iDroneFSM();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iDroneNode");
    ros::NodeHandle nh;

    for(int i = 0; i<NUM_WALL_MARKINGS; i++) {
        model.wallMarkings[i].hasBeenVisited = false;

        int counter = 0;
        int temp = i;
        while (temp > 4){
            counter++;
            temp = temp - 5;
        }

        std::stringstream ss;

        ss << "W0" << counter << ".0" << (i % 5) << std::endl;
        model.wallMarkings[i].id = ss.str();

    }
    model.qrSpotted = "";
    model.hasCalibrated = false;


    cv::namedWindow("view");
    cv::startWindowThread();

    ros::Subscriber navdata_sub = nh.subscribe("ardrone/navdata", 1000, navdataHandler);
    ros::Subscriber frontImageRaw_sub = nh.subscribe("ardrone/front/image_raw", 1000, imageCallback);
    ros::Subscriber wallQR_sub = nh.subscribe("wall_qr", 1000, wallQRHandler);
    ros::Subscriber qrSpotted_sub = nh.subscribe("qr_spotted", 1000, qrSpottedHandler);

    takeoff_pub = nh.advertise<std_msgs::Empty>("ardrone/takeoff", 1000);
    land_pub = nh.advertise<std_msgs::Empty>("ardrone/land", 1000);
    reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset", 1000);
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    std::thread FSMThread(iDroneFSM);

    /*
    for(int i = 0; i< NUM_WALL_MARKINGS; i++){
        std::cout << model.wallMarkings[i].id << std::endl;
    }*/

    ros::spin();
    run = 0;

    cv::destroyWindow("view");
    FSMThread.join();
    std::cout << "så er vi her!" << std::endl;
}


//====== Function implementations ======
void navdataHandler(ardrone_autonomy::Navdata in_navdata){
    std::cout << "navData: " << in_navdata.state << std::endl;

    navLock.lock();
    model.navdata = in_navdata;
    fsm.update(model);
    navLock.unlock();
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

void wallQRHandler(iDrone::qrAdjust msg){
    std::cout << "qr wall msg recieved" << std::endl;

    navLock.lock();
    model.qrAdjust = msg;
    fsm.update(model);
    navLock.unlock();
}

void qrSpottedHandler(const std_msgs::String::ConstPtr& msg){
    std::cout << "qr spotted recieved" << std::endl;

    navLock.lock();
    model.qrSpotted = msg->data.c_str();


    fsm.update(model);

    model.qrSpotted = "";
    navLock.unlock();
}

void iDroneFSM() {

    while (run) {

        fsm.act(model);

        //std::this_thread::sleep_for(std::chrono::seconds(2));

    }
}

void ControlPanel::takeOff(){
    pubLock.lock();
    takeoff_pub.publish(std_msgs::Empty());
    pubLock.unlock();
}

void ControlPanel::hover(){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0;
    cmdT.linear.z = 0;
    cmdT.linear.x = 0;
    cmdT.linear.y = 0;

    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::spinLeft(){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 1;
    cmdT.linear.z = 0;
    cmdT.linear.x = 0;
    cmdT.linear.y = 0;
    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::spinRight(){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = -1;
    cmdT.linear.z = 0;
    cmdT.linear.x = 0;
    cmdT.linear.y = 0;
    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::goLeft(){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0;
    cmdT.linear.z = 0;
    cmdT.linear.x = 0;
    cmdT.linear.y = 1;

    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::goRight(){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0;
    cmdT.linear.z = 0;
    cmdT.linear.x = 0;
    cmdT.linear.y = -1;

    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::land(){
    pubLock.lock();
    land_pub.publish(std_msgs::Empty());
    pubLock.unlock();
}

void ControlPanel::up(){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0;
    cmdT.linear.z = 1;
    cmdT.linear.x = 0;
    cmdT.linear.y = 0;

    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::down(){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0;
    cmdT.linear.z = -1;
    cmdT.linear.x = 0;
    cmdT.linear.y = 0;

    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::forward(){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0;
    cmdT.linear.z = 0;
    cmdT.linear.x = 1;
    cmdT.linear.y = 0;

    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::backward(){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0;
    cmdT.linear.z = 0;
    cmdT.linear.x = -1;
    cmdT.linear.y = 0;

    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::reset() {
    pubLock.lock();
    reset_pub.publish(std_msgs::Empty());
    pubLock.unlock();
}
