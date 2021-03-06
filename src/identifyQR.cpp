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
#include <iDrone/qrAdjust.h>
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
    ros::Subscriber bottomImageRaw_sub = nh.subscribe("ardrone/bottom/image_raw", 1, imageCallback);
    ros::Subscriber frontImageRaw_sub = nh.subscribe("ardrone/front/image_raw", 1, imageCallback);
    //PC camera
    //ros::Subscriber frontImageRaw_sub = nh.subscribe("camera/image_raw", 5, imageCallback);
    QRData = nh.advertise<iDrone::qrAdjust>("QRinfo", 1);
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

    iDrone::qrAdjust qrOut;
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

    line(cv_ptr->image,Point(width/2,height),Point(width/2,0),Scalar(0,0,0),1);
    line(cv_ptr->image,Point(width,height/2),Point(0,height/2),Scalar(0,0,0),1);

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
    int32_t largestQR = 0;
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


        //Keeps QR upright for proper data.

        vector<Point> sortedPointsByNumber;
        sortedPointsByNumber = vp;
        bool unordered = true;
        Point tempPoint;

        for (int i = 0 ; i<sortedPointsByNumber.size();i++){
            for (int k = 0 ; k<sortedPointsByNumber.size()-1 ; k++){
            if(sortedPointsByNumber[k].y > sortedPointsByNumber[k+1].y ) {
                tempPoint = sortedPointsByNumber[k+1];
                sortedPointsByNumber[k+1] = sortedPointsByNumber[k];
                sortedPointsByNumber[k] = tempPoint;
            }
            }
        }

        vector<Point> sortedPoints (4);


        if(sortedPointsByNumber[0].x < sortedPointsByNumber[1].x) {
            sortedPoints[0] = sortedPointsByNumber[0];
            sortedPoints[3] = sortedPointsByNumber[1];
        }
        else{
            sortedPoints[3] = sortedPointsByNumber[0];
            sortedPoints[0] = sortedPointsByNumber[1];
        }

        if(sortedPointsByNumber[2].x < sortedPointsByNumber[3].x) {
            sortedPoints[1] = sortedPointsByNumber[2];
            sortedPoints[2] = sortedPointsByNumber[3];
        }
        else {
            sortedPoints[2] = sortedPointsByNumber[2];
            sortedPoints[1] = sortedPointsByNumber[3];
        }




        vp=sortedPoints;


        // Calculate center of QR  code
        centerx = (vp[0].x + vp[1].x + vp[2].x + vp[3].x)/4;
        centery = (vp[0].y + vp[1].y + vp[2].y + vp[3].y)/4;


        // calculate the lengths of the sides and draw lines.
        for(int i=0;i<vp.size();i++) {

            if (i == 0) {
                leftLength = sqrt(pow(vp[i].x - vp[(i + 1) % 4].x, 2) + pow(vp[i].y - vp[(i + 1) % 4].y, 2));
                line(cv_ptr->image,vp[i],vp[(i+1)%4],Scalar(255,0,0),3);
                cout << "leftlength: " << leftLength << endl;
            }
            else if (i == 1) {
                botLength = sqrt(pow(vp[i].x - vp[(i + 1) % 4].x, 2) + pow(vp[i].y - vp[(i + 1) % 4].y, 2));
                line(cv_ptr->image,vp[i],vp[(i+1)%4],Scalar(0,255,0),3);
                cout << "botLength: " << botLength << endl;
            }
            else if (i == 2) {
                rightLength = sqrt(pow(vp[i].x - vp[(i + 1) % 4].x, 2) + pow(vp[i].y - vp[(i + 1) % 4].y, 2));
                line(cv_ptr->image,vp[i],vp[(i+1)%4],Scalar(0,0,255),3);
                cout << "rightlength: " << rightLength << endl;
            }
            else if (i == 3) {
                topLength = sqrt(pow(vp[i].x - vp[(i + 1) % 4].x, 2) + pow(vp[i].y - vp[(i + 1) % 4].y, 2));
                line(cv_ptr->image,vp[i],vp[(i+1)%4],Scalar(0,0,0),3);
                cout << "toplength: " << topLength << endl;
            }

            circle(cv_ptr->image, Point(centerx,centery), 4, Scalar(0,0,255), 2, 8, 0);

        }


        //===== CALCULATE WHICH QR IS THE LARGEST =====
        decodedQr = symbol->get_data();

        int32_t QRSize = leftLength + botLength + rightLength + topLength;
        if(QRSize > largestQR){
            largestQR = QRSize;
            qrOut.qr_id = decodedQr;
            qrSpotted.data = decodedQr;
            qrOut.l_height = leftLength;
            qrOut.b_length = botLength;
            qrOut.r_height = rightLength;
            qrOut.t_length = topLength;
            qrOut.c_pos = (float)(centerx - (width/2)) / (width/2);;
        }

        qrcodecenters.push_back(centerx);

        putText(cv_ptr->image, decodedQr, Point(250, 435), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8);
        qrcodes++;

    }


    // THIS IS OLD, KEPT ONLY AS A RELIC!!!! *SOOO SHIIIINY!*
/*    if(qrcodes > 0){
        cout << qrcodes << endl;
        int centerxtemp = numeric_limits<int>::max();
        float c_pos;


        for (int i=0; i<qrcodecenters.size(); i++){
            if(qrcodecenters[i] < centerxtemp)
                centerxtemp = qrcodecenters[i];
        }

        c_pos = (float)(centerxtemp - (width/2)) / (width/2);
        qrOut.c_pos = c_pos;
    }*/





    QRData.publish(qrOut);
    QRSpotted.publish(qrSpotted);


    imshow("Image",cv_ptr->image);
    // clean up
    qrImage.set_data(NULL, 0);
    waitKey(3);
}
