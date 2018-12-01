#include <mutex>
#include <thread>
#include <condition_variable>
#include <unordered_map>
#include <iostream>
#include <ros/ros.h>
// Import the two types of messages we're interested in
#include <sensor_msgs/Image.h> // for receiving the video feed
#include <ardrone_autonomy/Navdata.h> // for receiving navdata feedback
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include "drone_status.h" // An enumeration of Drone Statuses

using namespace std;
using namespace cv;

Mat image;

bool newImageArrived;
bool guiThreadExit;
mutex lastArrivedImageMutex;
Mat lastArrivedImage;
condition_variable imageConVar;

void ReceiveImage(const sensor_msgs::ImageConstPtr& rosImage)
{
	static int numImage = 0;
	numImage++;
	cout << "Image received "  << numImage << endl;
	cv_bridge::CvImageConstPtr cvImage(cv_bridge::toCvShare(rosImage, "bgr8"));
	//cv::imwrite(string("/home/kartal/Desktop/ardrone_images/burst/photo")+ to_string(numImage) + ".jpg", cvImage->image);
	unique_lock<mutex> lock(lastArrivedImageMutex);
	lastArrivedImage = cvImage->image;
	newImageArrived = true;
	imageConVar.notify_all();
}

unordered_map<DroneStatus, const char *> StatusMessages = {
	{DroneStatus::Emergency , "Emergency"},
	{DroneStatus::Inited    , "Initialized"},
	{DroneStatus::Landed    , "Landed"},
	{DroneStatus::Flying    , "Flying"},
	{DroneStatus::Hovering  , "Hovering"},
	{DroneStatus::Test      , "Test (?)"},
	{DroneStatus::TakingOff , "Taking Off"},
	{DroneStatus::GotoHover , "Going to Hover Mode"},
	{DroneStatus::Landing   , "Landing"},
	{DroneStatus::Looping   , "Looping (?)"}
};

void ReceiveNavdata(const ardrone_autonomy::NavdataConstPtr& navdata)
{
	auto smsgIt = StatusMessages.find(static_cast<DroneStatus>(navdata->state));
	if (smsgIt == StatusMessages.end())
		cout << "Unknown Status" << ' ';
	else
		cout << smsgIt->second << ' ';
	cout << "(Battery: " << navdata->batteryPercent << "%)" << endl;
}


Mat readLastArrivedImage()
{
	unique_lock<mutex> lock(lastArrivedImageMutex);
	imageConVar.wait(lock, []{return newImageArrived || guiThreadExit;});
	Mat img(lastArrivedImage);
	return img;
}

void guiThreadEntry()
{
	namedWindow( "Hough Circle Ball Detector", 0 );
	while (1)
	{
		cv::Mat bgr_image = readLastArrivedImage();
		if (guiThreadExit) // readLastArrivedImage stops waiting for a new image if guiThreadExit has been set by the main thread.
			break;
		assert(bgr_image.empty() == false);
		// This part is from https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/
		cv::Mat orig_image = bgr_image.clone();
		cv::medianBlur(bgr_image, bgr_image, 3);
		// Convert input image to HSV
		cv::Mat hsv_image;
		cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);
		// Threshold the HSV image, keep only the red pixels
		cv::Mat lower_red_hue_range;
		cv::Mat upper_red_hue_range;
		cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
		cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
		// Combine the above two images
		cv::Mat red_hue_image;
		cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
		cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);
		// Use the Hough transform to detect circles in the combined threshold image
		std::vector<cv::Vec3f> circles;
		cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 10, 0, 0);
		for(size_t current_circle = 0; current_circle < circles.size() && current_circle == 0; ++current_circle) {
			cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
			int radius = std::round(circles[current_circle][2]);
			cv::circle(orig_image, center, radius, cv::Scalar(0, 255, 0), 5);
		}
		imshow( "Hough Circle Ball Detector", orig_image );
		waitKey(1);
	}
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ardrone_opencv_demo");
    ros::NodeHandle nh;
    ros::Subscriber subNavdata = nh.subscribe<ardrone_autonomy::Navdata>("/ardrone/navdata", 10, ReceiveNavdata);
    ros::Subscriber subVideo = nh.subscribe<sensor_msgs::Image>("/ardrone/image_raw", 10, ReceiveImage);

    thread guiThread(guiThreadEntry);

    ros::spin();

    if (true)  // Limit the scope of *lock*.
    {
        unique_lock<mutex> lock(lastArrivedImageMutex);
        guiThreadExit = true;
        imageConVar.notify_all();
    }
    guiThread.join();

    return 0;
}

