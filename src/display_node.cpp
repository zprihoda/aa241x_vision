/**
 * Node to display a live video feed of images broadcast to the 'image' topic.
 */


#include <iostream>
#include <string>
#include <vector>
#include <chrono>	// std:c:chrono::system_clock
#include <ctime>	// std::time_t, struct std::tm, std::localtime
#include <iomanip>  // std::put_time

// system includes
#include <sys/stat.h>	// directory handling
#include <sys/types.h>	// directory handling
#include <sys/time.h>	// time functions


// specific library stuff
#include <opencv2/opencv.hpp>	// opencv
#include <ros/ros.h> 			// ros

// image handling
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace std;
using namespace cv;


/**
 * The node contents in wrapped into a class to allow for easier handling of
 * shared information between the callbacks and main loop
 */
class DisplayNode {

public:


	/**
	 * Constructor for the display node
	 * @param  frame_width  the integer width of the image frame in pixels
	 * @param  frame_height the integer height of the image frame in pixels
	 */
	DisplayNode(int frame_width, int frame_height);

	/**
	 * the main loop to be run for this node (called by the `main` function)
	 * @return exit code
	 */
	int run();

private:

	// node handler and image transport handler
	ros::NodeHandle _nh;
	image_transport::ImageTransport _it;

	// settings
	int _frame_width;	// image width in pixels
	int _frame_height;	// image height in pixels

	// subscriber
	image_transport::Subscriber _image_sub;	// subscriber to the image topic

	// callbacks

	/**
	 * callback for image topic data
	 * @param msg pointer to the published imaged data
	 */
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);

};



DisplayName::DisplayNode(int frame_width, int frame_height) :
_frame_width(frame_width),
_frame_height(frame_height),
_it(_nh)
{
	// subscribe to the image with the following information:
	// 		- the topic name is "image"
	// 		- the queue size is 1 (only the most recent published data is saved)
	// 		- the function to call upon receiving an image is `imageCallback`
	_image_sub = _it.subscribe("image", 1, &DisplayNode::imageCallback, this);
}


void DisplayNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

	// wrap the decoding in a try catch to handle the error in the encoding
	try {
		// get the image
		cv::Mat image = cv_bridge::toCvShare(msg, "mono8")->image;
		cv::imshow("debug", image);  // show the image to the window called "debug"

	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
	}
}


int DisplayNode::run() {

	// create the window
	cv::namedWindow("debug", cv::WINDOW_NORMAL);
	cv::resizeWindow("debug", _frame_width, _frame_height);
	cv::startWindowThread();

	// spin -> this listens for the callbacks and calls them as needed
	ros::spin();

	// destroy the window when the node is stopped
	cv::destroyWindow("debug");
}


// the main function


int main(int argc, char **argv) {

	// define the node -> specify the registered name
	ros::init(argc, argv, "display_node");

	// get parameters from the launch file which define some mission
	ros::NodeHandle private_nh("~");
	int frame_width, frame_height;

	private_nh.param("frame_width", frame_width, 640);
	private_nh.param("frame_height", frame_height, 512);

	// create the node with the desired settings
	DisplayNode node(frame_width, frame_height);

	// run the node
	return node.run();
}
