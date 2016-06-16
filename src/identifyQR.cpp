#include "zbar.h"  
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <cmath>
#include <cstdio>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <limits>
#include <zbarQR/qrAdjust.h>
#include <string>
#include <std_msgs/String.h>

 using namespace std;
 using namespace zbar;  
 using namespace cv;
namespace enc = sensor_msgs::image_encodings;
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
ImageScanner scanner;
ros::Publisher QRData;
ros::Publisher QRSpotted;

int main(int argc, char **argv){
     //Init QR scanner
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    // ROS
     ros::init(argc, argv, "image_listener");
     ros::NodeHandle nh;
     cv::namedWindow("Image");
     //cv::namedWindow("Traces");
     //cv::namedWindow("QR code");
     cv::startWindowThread();

    image_transport::ImageTransport it(nh);

    //Front camera
    //ros::Subscriber frontImageRaw_sub = nh.subscribe("ardrone/front/image_raw", 10, imageCallback);
    //PC camera
    ros::Subscriber frontImageRaw_sub = nh.subscribe("camera/image_raw", 5, imageCallback);
    QRData = nh.advertise<zbarQR::qrAdjust>("wall_qr", 1);
    QRSpotted = nh.advertise<std_msgs::String>("qr_spotted", 1);

    /* Toggle to downward camera, show downward.
	//ros::ServiceClient toggleCam_srv;
	toggleCam_srv        = nh.serviceClient<std_srvs::Empty>(nh.resolveName("ardrone/togglecam"),1);

	std_srvs::Empty toggleCam_srv_srvs;
	toggleCam_srv.call(toggleCam_srv_srvs);

	ros::Subscriber frontImageRaw_sub = nh.subscribe("ardrone/bottom/image_raw", 10, imageCallback);
	 */

    ros::spin();
    cv::destroyWindow("Image");
    cv::destroyWindow("Traces");
    cv::destroyWindow("QR code");



 }


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    zbarQR::qrAdjust qrOut;
    std_msgs::String qrSpotted;
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);

    // obtain image data
    /*char file[256];
    cin>>file;
    Mat img = imread(file,0);
    */

    Mat imgout;
    cvtColor(cv_ptr->image,imgout,CV_BGR2GRAY);
    int width = cv_ptr->image.cols;
    int height = cv_ptr->image.rows;

    int32_t topLength;
    int32_t botLength;
    int32_t leftLength;
    int32_t rightLength;

    uchar *raw = (uchar *)imgout.data;
    // wrap image data
    Image qrImage(width, height, "Y800", raw, width * height);
    // scan the image for barcodes
    int n = scanner.scan(qrImage);
    int centerx,centery;
    // extract results
    string decodedQr;
    int qrcodes = 0;
    vector<int> qrcodecenters;

    for(Image::SymbolIterator symbol = qrImage.symbol_begin();
        symbol != qrImage.symbol_end();
        ++symbol) {

        vector<Point> vp;
        // do something useful with results
        cout << "decoded " << symbol->get_type_name()
        << " symbol \"" << symbol->get_data() << '"' <<" "<< endl;
        int n = symbol->get_location_size();
        for(int i=0;i<n;i++){
            vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
        }

        centerx = (vp[0].x + vp[1].x + vp[2].x + vp[3].x)/4;
        centery = (vp[0].y + vp[1].y + vp[2].y + vp[3].y)/4;

        for(int i=0;i<vp.size();i++) {

            if (i == 0) {
                botLength = sqrt(pow(vp[i].x - vp[(i + 1) % 4].x, 2) + pow(vp[i].y - vp[(i + 1) % 4].y, 2));
                qrOut.b_length = botLength;
                cout << "botLength: " << botLength << endl;
            }
            else if (i == 1) {
                leftLength = sqrt(pow(vp[i].x - vp[(i + 1) % 4].x, 2) + pow(vp[i].y - vp[(i + 1) % 4].y, 2));
                qrOut.l_height = leftLength;
                cout << "leftLength: " << leftLength << endl;
            }
            else if (i == 2) {
                topLength = sqrt(pow(vp[i].x - vp[(i + 1) % 4].x, 2) + pow(vp[i].y - vp[(i + 1) % 4].y, 2));
                qrOut.t_length = topLength;
                cout << "topLength: " << topLength << endl;
            }
            else if (i == 3) {
                rightLength = sqrt(pow(vp[i].x - vp[(i + 1) % 4].x, 2) + pow(vp[i].y - vp[(i + 1) % 4].y, 2));
                qrOut.r_height = rightLength;
                cout << "rightLength: " << rightLength << endl;
            }

            circle(cv_ptr->image, Point(centerx,centery), 4, Scalar(0,0,255), 2, 8, 0);
            line(cv_ptr->image,vp[i],vp[(i+1)%4],Scalar(255,0,0),3);
        }


        qrcodecenters.push_back(centerx);


        decodedQr = symbol->get_data();
        qrOut.qr_id = decodedQr;
        qrSpotted.data = decodedQr;
        putText(cv_ptr->image, decodedQr, Point(250, 435), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8);
        qrcodes++;
    }
    if(qrcodes > 0){
        cout << qrcodes << endl;
        int centerxtemp = numeric_limits<int>::max();
        float c_pos;
        for (int i=0; i<qrcodecenters.size(); i++){
            if(qrcodecenters[i] < centerxtemp)
                centerxtemp = qrcodecenters[i];
        }



        c_pos = (float)(centerxtemp - (width/2)) / (width/2);
        qrOut.c_pos = c_pos;

    }

    if(!(decodedQr == "")){
        QRData.publish(qrOut);
        QRSpotted.publish(qrSpotted);
    }

    imshow("Image",cv_ptr->image);
    // clean up
    qrImage.set_data(NULL, 0);
    waitKey(3);
}
