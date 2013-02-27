
#include "cv.h"
#include "highgui.h"
#include <stdio.h>
//#include "capture.h"
#include "facedet.h"
#include "camshift_wrapper.h"

#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include <message_filters/time_synchronizer.h>

#include <boost/thread.hpp>



//// Constants
const char * DISPLAY_WINDOW = "DisplayWindow";
namespace enc = sensor_msgs::image_encodings;



//// Global variables
IplImage  * pVideoFrameCopy = 0;
CvRect * pFaceRect = 0;
CvRect rect;
void imageCb_rgb(const sensor_msgs::ImageConstPtr& msg);
void face_detected_callback(const std_msgs::Int32MultiArray::ConstPtr& array);
int initAll();
void exitProgram(int code);
void captureVideoFrame();
IplImage frame;
boost::mutex face_mutex_,image_mutex_;
void face_detected_callback(const std_msgs::Int32MultiArray::ConstPtr& array){


	face_mutex_.lock();
	rect.height = array->data[0];
	rect.width = array->data[1];
	rect.y = array->data[2];
	rect.x = array->data[3];
	pFaceRect = &rect;
	face_mutex_.unlock();

}
void imageCb_rgb(const sensor_msgs::ImageConstPtr& msg)
{
	image_mutex_.lock();
	cv_bridge::CvImagePtr cv_ptr;
	try{cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);}
	catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());return;}
	frame = cv_ptr->image;
	//	pVideoFrameCopy = &frame;
	IplImage  * pVideoFrame;
	pVideoFrame = &frame;
	pVideoFrameCopy = cvCreateImage( cvGetSize(pVideoFrame), 8, 3 );
	cvCopy( &frame, pVideoFrameCopy, 0 );


	if(!pFaceRect || pFaceRect->height==0){
		pFaceRect = 0;
		cvShowImage( DISPLAY_WINDOW, pVideoFrameCopy );
		cv::waitKey(3);
		printf("no face\n");
	}
	else{
		startTracking(pVideoFrameCopy, pFaceRect);
		CvBox2D faceBox;
		faceBox = track(pVideoFrameCopy);
		// outline face ellipse
		cvEllipseBox(pVideoFrameCopy, faceBox,
				CV_RGB(255,0,0), 3, CV_AA, 0 );
		cvShowImage( DISPLAY_WINDOW, pVideoFrameCopy );
		cv::waitKey(3);
		printf(">%d %d %d %d<\n",pFaceRect->x,pFaceRect->y,pFaceRect->width,pFaceRect->height);
		printf("detected face x:%d y:%d w:%d h:%d)\n",pFaceRect->x,pFaceRect->y,pFaceRect->width,pFaceRect->height);
	}
	image_mutex_.unlock();
}


//////////////////////////////////
// main()
//


int main( int argc, char** argv )
{
	//	CvRect * pFaceRect = 0;
	if( !initAll() ) exitProgram(-1);

	ros::init(argc, argv, "listener2");
	ros::NodeHandle n;

	ros::Subscriber sub3 = n.subscribe("/sn_kinect1/rgb/image_color", 1, imageCb_rgb);
	ros::Subscriber sub = n.subscribe("/face_detected", 1, face_detected_callback);
	ros::Rate r(20);
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
//////////////////////////////////
// initAll()
//
int initAll()
{
//	if( !initCapture() ) return 0;
	if( !initFaceDet("ex1_face_trace/haarcascade_frontalface_default.xml"))
		return 0;

	// Startup message tells user how to begin and how to exit
	printf( "\n********************************************\n"
			"To exit, click inside the video display,\n"
			"then press the ESC key\n\n"
			"Press <ENTER> to begin"
			"\n********************************************\n" );
	fgetc(stdin);

	// Create the display window
	cvNamedWindow( DISPLAY_WINDOW, 1 );

	// Initialize tracker
	if( !createTracker(pVideoFrameCopy) ) return 0;

	// Set Camshift parameters
	setVmin(60);
	setSmin(50);

	return 1;
}


//////////////////////////////////
// exitProgram()
//
void exitProgram(int code)
{
	// Release resources allocated in this file
	cvDestroyWindow( DISPLAY_WINDOW );
	cvReleaseImage( &pVideoFrameCopy );

	// Release resources allocated in other project files
	closeFaceDet();
	releaseTracker();

	exit(code);
}




