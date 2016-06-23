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
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "iDrone/CamSelect.h"

#include <stdint.h>


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

ros::ServiceClient flatTrimClient;
ros::ServiceClient cam_srv;

iDrone::CamSelect camSelect_srv;

float TurnSpeed = 0.50;
float MoveSpeed = 0.1;

//====== Function prototypes ======
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void navdataHandler(ardrone_autonomy::Navdata in_navdata);
void wallQRHandler(iDrone::qrAdjust msg);
void qrSpottedHandler(const std_msgs::String::ConstPtr& msg);
void selectiveImageAnalysisCallback(const sensor_msgs::ImageConstPtr& msg);
void floorAFHandler(iDrone::afAdjust msg);
void ORBhandler(iDrone::afAdjust msg);

void iDroneFSM();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iDroneNode");
    ros::NodeHandle nh;

    camSelect_srv.request.channel = 0; // Camera is set to front (1 == bottom)

    //init model
    iDrone::qrAdjust qrAdjust;
    qrAdjust.qr_id = "";
    qrAdjust.l_height = 0;
    qrAdjust.r_height = 0;
    qrAdjust.t_length = 0;
    qrAdjust.b_length = 0;
    qrAdjust.c_pos = 0;

    model.qrAdjust = qrAdjust;

    for(int i = 0; i<NUM_WALL_MARKINGS; i++) {
        model.wallMarkings[i].hasBeenVisited = false;

        int counter = 0;
        int temp = i;
        while (temp > 4){ // 5 wallmarkings on each wall
            counter++;
            temp = temp - 5;
        }

        std::stringstream ss;

        ss << "W0" << counter << ".0" << (i % 5);
        model.wallMarkings[i].id = ss.str();

    }

    model.wallMarkings[W00_e].hasBeenVisited = true;
    model.wallMarkings[W01_e].hasBeenVisited = true;
    model.wallMarkings[W23_e].hasBeenVisited = true;
    model.wallMarkings[W24_e].hasBeenVisited = true;
    model.wallMarkings[W30_e].hasBeenVisited = true;
    model.wallMarkings[W31_e].hasBeenVisited = true;
    model.wallMarkings[W32_e].hasBeenVisited = true;
    model.wallMarkings[W33_e].hasBeenVisited = true;
    model.wallMarkings[W34_e].hasBeenVisited = true;

    for(int j = 0; j<NUM_AIRFIELDS; j++){
        model.airfields[j].hasLanded = false;

        std::stringstream airss;

        airss << "AF.0" << j;
        model.airfields[j].airfieldQR = airss.str();

        std::cout << model.airfields[j].airfieldQR << std::endl;

        model.airfields[j].wallMarking = -1;
    }

    model.qrSpotted = "";
    model.hasCalibrated = false;

    for (int i = 0; i < NUM_AIRFIELDS; i++) {
        model.airfields[i].hasLanded = false;
        model.airfields[i].wallMarking = -1;
        model.airfields[i].x = -1;
        model.airfields[i].y = -1;
    }

    model.afAdjust.c_x = -1;
    model.afAdjust.c_y = -1;
    model.afAdjust.imgc_x = -1;
    model.afAdjust.imgc_y = -1;
    model.afAdjust.match = NO_MATCH_e;

    model.nextAirfield = AF2_e;
    model.currentWallMarking = -1;


    model.badMatchCounter = 0;
    model.consecutiveMatchesCounter = 0;

    //cv::namedWindow("view");
    //cv::startWindowThread();

    ros::Subscriber navdata_sub = nh.subscribe("ardrone/navdata", 1, navdataHandler);
    ros::Subscriber wallQR_sub = nh.subscribe("QRinfo", 1, wallQRHandler);
    ros::Subscriber qrSpotted_sub = nh.subscribe("qr_spotted", 1, qrSpottedHandler);
    //ros::Subscriber frontImageRaw_sub = nh.subscribe("ardrone/front/image_raw", 1, selectiveImageAnalysisCallback);
    //ros::Subscriber bottomImageRaw_sub = nh.subscribe("ardrone/bottom/image_raw", 1, selectiveImageAnalysisCallback);
    //ros::Subscriber floorAF_sub = nh.subscribe("ORB_Detection", 1, ORBhandler);
    ros::Subscriber floorAF2_sub = nh.subscribe("circlecoordinate", 1, floorAFHandler);

    flatTrimClient = nh.serviceClient<std_srvs::Empty>("ardrone/flatTrim", 1);

    takeoff_pub = nh.advertise<std_msgs::Empty>("ardrone/takeoff", 1000);
    land_pub = nh.advertise<std_msgs::Empty>("ardrone/land", 1000);
    reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset", 1000);
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    std::thread FSMThread(iDroneFSM);

    /*
    for(int i = 0; i< NUM_WALL_MARKINGS; i++){
        std::cout << model.wallMarkings[i].id << std::endl;
    }*/

    cam_srv = nh.serviceClient<iDrone::CamSelect>("/ardrone/setcamchannel");

    ros::spin();

    std::cout << "shutting down" << std::endl;

    run = 0;

    //cv::destroyWindow("view");
    FSMThread.join();
    std::cout << "så er vi her!" << std::endl;
}


//====== Function implementations ======
void navdataHandler(ardrone_autonomy::Navdata in_navdata){
    //std::cout << "navData: " << in_navdata.state << std::endl;

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
    /*std::cout << "qr wall msg recieved" << std::endl;
    std::cout << msg.r_height << std::endl;
    std::cout << msg.l_height << std::endl;
    std::cout << msg.t_length << std::endl;
    std::cout << msg.b_length << std::endl;
    std::cout << msg.c_pos << std::endl;
    std::cout << msg.qr_id << std::endl;*/

    navLock.lock();
    model.qrAdjust = msg;
    fsm.update(model);
    navLock.unlock();
}

void qrSpottedHandler(const std_msgs::String::ConstPtr& msg){


    navLock.lock();
    model.qrSpotted = msg->data;

    //std::cout << "qr spotted recieved: "  << model.qrSpotted << std::endl;

    if(fsm.currentState == SEARCH_e){
        for(int i = 0; i<NUM_AIRFIELDS; i++){
            if(model.airfields[i].airfieldQR == model.qrSpotted &&
                    i != model.nextAirfield){
                model.airfields[i].wallMarking = model.currentWallMarking;
                //TODO coordinates
            }
        }
    }

    if(model.currentWallMarking < 0){
        for(int i = 0; i<NUM_WALL_MARKINGS; i++){
            if(model.wallMarkings[i].id == model.qrSpotted){
                model.currentWallMarking = i;
            }
        }
    }

    fsm.update(model);

    model.qrSpotted = "";
    navLock.unlock();
}

void selectiveImageAnalysisCallback(const sensor_msgs::ImageConstPtr& msg){
    //TODO publish required imageanalysis topics
}

void floorAFHandler(iDrone::afAdjust msg){
    navLock.lock();

    if(model.afAdjust.match > 1 && msg.match > 1){//2 consecutive matches
        model.consecutiveMatchesCounter++;
    }

    model.afAdjust = msg;

    model.badMatchCounter++;

    if(model.afAdjust.match > 1){
        model.badMatchCounter = 0;
    }
/*
    std::cout << "Air Field match: "  << model.afAdjust.match << std::endl;
    std::cout << "c_x : "  << model.afAdjust.c_x << std::endl;
    std::cout << "c_y: "  << model.afAdjust.c_y << std::endl;
    std::cout << "imgc_x: "  << model.afAdjust.imgc_x << std::endl;
    std::cout << "imgc_y: "  << model.afAdjust.imgc_y << std::endl;

    float delta_x = model.afAdjust.c_x - model.afAdjust.imgc_x;
    float delta_y = model.afAdjust.c_y - model.afAdjust.imgc_y;

    std::cout << "delta x: "  << delta_x << std::endl;
    std::cout << "delta y: "  << delta_y << std::endl;
*/
    fsm.update(model);
    navLock.unlock();
}

void ORBhandler(iDrone::afAdjust msg){
    navLock.lock();
    //if(model.navdata.altd > 1700) {
        model.afAdjust = msg;
/*
    std::cout << "Air Field match: "  << model.afAdjust.match << std::endl;
    std::cout << "c_x : "  << model.afAdjust.c_x << std::endl;
    std::cout << "c_y: "  << model.afAdjust.c_y << std::endl;
    std::cout << "imgc_x: "  << model.afAdjust.imgc_x << std::endl;
    std::cout << "imgc_y: "  << model.afAdjust.imgc_y << std::endl;

    float delta_x = model.afAdjust.c_x - model.afAdjust.imgc_x;
    float delta_y = model.afAdjust.c_y - model.afAdjust.imgc_y;

    std::cout << "delta x: "  << delta_x << std::endl;
    std::cout << "delta y: "  << delta_y << std::endl;
*/
        fsm.update(model);
    //}
    navLock.unlock();
}

void iDroneFSM() {

    while (run) {
        if (navLock.try_lock()) {
            fsm.act(model);
            navLock.unlock();
        }


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
    cmdT.angular.z = TurnSpeed;
    cmdT.linear.z = 0;
    cmdT.linear.x = 0;
    cmdT.linear.y = 0;
    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::spinRight(){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = (-TurnSpeed);
    cmdT.linear.z = 0;
    cmdT.linear.x = 0;
    cmdT.linear.y = 0;
    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::goLeft(float speed){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0;
    cmdT.linear.z = 0;
    cmdT.linear.x = 0;
    cmdT.linear.y = speed;

    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::goRight(float speed){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0;
    cmdT.linear.z = 0;
    cmdT.linear.x = 0;
    cmdT.linear.y = (-speed);

    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::land(){
    pubLock.lock();
    land_pub.publish(std_msgs::Empty());
    pubLock.unlock();
}

void ControlPanel::up(float speed){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0;
    cmdT.linear.z = speed;
    cmdT.linear.x = 0;
    cmdT.linear.y = 0;

    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::down(float speed){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0;
    cmdT.linear.z = (-speed);
    cmdT.linear.x = 0;
    cmdT.linear.y = 0;

    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::forward(float speed){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0;
    cmdT.linear.z = 0;
    cmdT.linear.x = speed;
    cmdT.linear.y = 0;

    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::backward(float speed){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0;
    cmdT.linear.z = 0;
    cmdT.linear.x = (-speed);
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

bool hasCalledFlatTrim = false;
void ControlPanel::flatTrim() {
    if(hasCalledFlatTrim){
        return;
    }

    std::cout << "called flat trim" << std::endl;

    hasCalledFlatTrim = true;

    std_srvs::Empty flattrim_srv_srvs;

    pubLock.lock();
    flatTrimClient.call(flattrim_srv_srvs);
    pubLock.unlock();
}


void ControlPanel::frontCam() {
    if(camSelect_srv.request.channel == 0){
        return;
    }

    pubLock.lock();
    camSelect_srv.request.channel = (uint8_t) 0;

    cam_srv.call(camSelect_srv);
    pubLock.unlock();
}

void ControlPanel::bottomCam() {
    if(camSelect_srv.request.channel == 1){
        return;
    }

    pubLock.lock();
    camSelect_srv.request.channel = (uint8_t) 1;

    cam_srv.call(camSelect_srv);
    pubLock.unlock();
}

void ControlPanel::diagForwardRight(){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0;
    cmdT.linear.z = 0;
    cmdT.linear.x = MoveSpeed;
    cmdT.linear.y = (-MoveSpeed);

    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::diagBackwardRight(){
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = 0;
    cmdT.linear.z = 0;
    cmdT.linear.x = -1;
    cmdT.linear.y = -1;

    pubLock.lock();
    vel_pub.publish(cmdT);
    pubLock.unlock();
}

void ControlPanel::updateSearchState() {
    model.wallMarkings[model.currentWallMarking].hasBeenVisited = true;
    model.consecutiveMatchesCounter = 0;
}

void ControlPanel::updateNextAirfield() {

    model.airfields[model.nextAirfield].hasLanded = true;


    model.nextAirfield++;
    if(model.nextAirfield == NUM_AIRFIELDS){
        model.nextAirfield = AF1_e;
    }
}

void ControlPanel::updateFoundAirfield(int afID){
    model.airfields[afID].wallMarking = model.currentWallMarking;
}

void ControlPanel::updateCurrentWallmarking(int wallID) {
    model.currentWallMarking = wallID;
}
