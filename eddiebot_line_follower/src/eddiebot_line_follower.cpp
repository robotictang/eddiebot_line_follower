/*
 * Copyright (c) 2012, Tang Tiong Yew
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <geometry_msgs/Twist.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

/*here is a simple program which demonstrates the use of ros and opencv to do image manipulations on video streams given out as image topics from the monocular vision
 of robots,here the device used is a ardrone(quad-rotor).*/
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
static const char WINDOW[] = "Image window";

float prevVelocity_angular, prevVelocity_linear, newVelocity_angular,
		newVelocity_linear;
float derive_angular, derive_linear, dt = 0.5;
float horizontalcount;

class ImageConverter {
	ros::NodeHandle nh_, ph_;
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Publisher tog;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_; //image subscriber
	image_transport::Publisher image_pub_; //image publisher(we subscribe to ardrone image_raw)
	std_msgs::String msg;

	double linear_, angular_;
	double l_scale_, a_scale_;
	double left_threshold, right_threshold;
public:
	ImageConverter() :
		linear_(0.05),
		angular_(0.05),
		l_scale_(1.0),
		a_scale_(1.0),
		left_threshold(150),
		right_threshold(450),
		it_(nh_) {
			pub = n.advertise <geometry_msgs::Twist> ("cmd_vel", 1);
			image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
					&ImageConverter::imageCb, this);
			image_pub_ = it_.advertise("/arcv/Image", 1);

			// Initialize Parameters
			nh_.param("scale_angular", a_scale_, a_scale_);
			nh_.param("scale_linear", l_scale_, l_scale_);
			nh_.param("angular", angular_, angular_);
			nh_.param("linear", linear_, linear_);
			nh_.param("left_threshold", left_threshold, left_threshold);
			nh_.param("right_threshold", right_threshold, right_threshold);

	}

	~ImageConverter() {
		cv::destroyWindow(WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg) {

		sensor_msgs::CvBridge bridge; //we need this object bridge for facilitating conversion from ros-img to opencv
		IplImage* img = bridge.imgMsgToCv(msg, "rgb8"); //image being converted from ros to opencv using cvbridge
		geometry_msgs::Twist velMsg;
		CvMemStorage* storage = cvCreateMemStorage(0);
		CvSeq* lines = 0;
		int i, c, d;
		float c1[50];
		float m, angle;
		float buf;
		float m1;
		float dis;
		int k = 0, k1 = 0;
		int count = 0;

		float xv;
		float yv;
		int vc = 0;
		float xvan = 0, yvan = 0;
		static float xvan1 = 0, yvan1 = 0;
		float num = 0;
		static float count1 = 0;
		float dv;
		float vxx, vyy;

		cvSetImageROI(img, cvRect(0, 0, 640, 480));
		IplImage* out1 = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3); //make sure to feed the image(img) data to the parameters necessary for canny edge output
		IplImage* gray_out = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		IplImage* canny_out = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		IplImage* gray_out1 = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
		IplImage* canny_out1 = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		IplImage* canny_out2 = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);

		IplImage* gray_out2 = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);

		cvSmooth(img, out1, CV_GAUSSIAN, 11, 11);

		cvCvtColor(out1, gray_out, CV_RGB2GRAY);
		cvNot(gray_out,gray_out2);
		cvCanny(gray_out2, canny_out, 80, 125, 3);
		cvCvtColor(canny_out, gray_out1, CV_GRAY2BGR);

//		//////////////// Color Filtering //////////////
//
//		//IplImage *imgHsv,*imgResult;
//		int mR_val=255,mG_val=128,mB_val=64,MAR_val=128,MAG_val=0,MAB_val=0;//default green .ctrl BLUE to find color
//
//
//		IplImage* test=cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
//		IplImage* imgResult=cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
//		IplImage* imgHsv=cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
//
//		cvCvtColor( test, imgHsv, CV_BGR2HSV);//convert the color space
//		CvScalar min_color = CV_RGB(mR_val,mG_val,mB_val);
//		CvScalar max_color = CV_RGB(MAR_val,MAG_val,MAB_val);
//		cvInRangeS(imgHsv, min_color,max_color, imgResult);//search for the color in image
//		//////////////// Color Filtering //////////////



		lines = cvHoughLines2(canny_out, storage, CV_HOUGH_PROBABILISTIC, 1,
				CV_PI / 180, 80, 50, 10);
		for (i = 0; i < lines->total; i++) {

			CvPoint* line = (CvPoint*) cvGetSeqElem(lines, i);
			{
				{
					cvLine(out1, line[0], line[1], CV_RGB(0, 255, 0), 1, 8);
					cvLine(gray_out1, line[0], line[1], CV_RGB(0, 255, 0), 2,
							8);
					xv = line[0].x - line[1].x;
					yv = line[0].y - line[1].y;
//					velMsg.linear = atan2(xv, yv) * 180 / 3.14159265;
//					angle = velMsg.linear;
//					printf("test");
//					if (velMsg.linear < -90) {
//						printf("one");
//						velMsg.linear = velMsg.linear + 180;
//					}
//					buf = (line[0].x + line[1].x) / 2;
//
//					if (abs(85 - buf) <= 15) {
//						velMsg.angular = 0;
//						printf("two");
//					} else {
//						velMsg.angular = (85 - (line[0].x + line[1].x) / 2);
//						printf("three");
//					}

//					if (abs(velMsg.angular) > 50) {
//						velMsg.angular = 0;
//						printf("four");
//					}
// Modified
					angle = atan2(xv, yv) * 180 / 3.14159265;


					buf = (line[0].x + line[1].x) / 2;
					printf("line[0].x: %f", line[0].x);
					printf("line[1].x: %f", line[1].x);
					velMsg.linear.z = buf;
					velMsg.linear.y = angle;

					if (abs(buf) <= left_threshold) { // turn left
						velMsg.angular.z = angular_ * a_scale_;
					}
					else if (abs(buf) >= right_threshold) { // turn right
						velMsg.angular.z = -angular_ * a_scale_;
					}
					else {
						velMsg.linear.x = linear_ * l_scale_;
					}


					printf("\nX::Y::X1::Y1::%d:%d:%d:%d", line[0].x, line[0].y,
							line[1].x, line[1].y);

					pub.publish(velMsg);

				}

			}

			cvSeqRemove(lines, i);

		}
		cvShowImage("OUT1", out1);   //lines projected onto the real picture
		cvShowImage("GRAY_OUT1", gray_out1);

		// Added Display
//		cvShowImage("GRAY_OUT", gray_out);
//		cvShowImage("CANNY_OUT", canny_out);
//		cvShowImage("CANNY_OUT1", canny_out1);
//		cvShowImage("gray_out2", gray_out2);
//		cvShowImage("imgResult", imgResult);
		cv::waitKey(3);
		sensor_msgs::ImagePtr out = sensor_msgs::CvBridge::cvToImgMsg(img,
				"rgb8");   //image converted from opencv to ros for publishing
		image_pub_.publish(out);
		cvClearMemStorage(storage);
		cvReleaseMemStorage(&storage);
		cvReleaseImage( &out1 );
		cvReleaseImage( &gray_out );
		cvReleaseImage( &canny_out );
		cvReleaseImage( &gray_out1 );
		cvReleaseImage( &canny_out1 );
		cvReleaseImage( &canny_out2 );
		cvReleaseImage( &gray_out2 );



	}

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}
