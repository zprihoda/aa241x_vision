/**
 * this file will contain the skeleton / example node for reading from the picam
 * using the C++ library
 *
 * maybe I'll also have it demonstrate the ability to find the AprilTag in the
 * image
 *
 * TODO: figure out how much the latency is and see if it will really cause
 * problems.  I think if they have an estimator running for the position of the
 * truck bed, it will help with any latency??? -> actually probably not...
 */

#include <iostream>
#include <raspicam/raspicam_cv.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>

// TODO: remove aa241x from the node name
class AA241xVisionNode {

public:

	AA241xVisionNode(int frame_width, int frame_height, bool publish_image);


	// main loop
	int run();


private:

	// node handler
	ros::NodeHandle _nh;
	image_transport::ImageTransport _it;


	// settings, etc
	int _frame_width;		// image width to use in [px]
	int _frame_height;		// image height to use in [px]
	bool _publish_image;	// true if the image data should be published

	// camera stuff
	raspicam::RaspiCam_Cv _camera;	// TODO: use the correct class name here

	// april tag stuff
	//APDetector _detector;	// TODO: use the correct class name here

	// subscribers
	// TODO: figure out what subscriber may be desired


	// publishers
	ros::Publisher _tag_relative_position_pub;	// the relative position vector to the truck
	ros::Publisher _tag_details_pub;			// the raw tag details (for debugging)
	image_transport::Publisher _image_pub;

	// TODO: add a publisher of the image frame -> for the display node

	// callbacks
	// TODO: figure out what might be needed


	// helper functions
	// TODO: any helper functions here

};


AA241xVisionNode::AA241xVisionNode(int frame_width, int frame_height, bool publish_image) :
_frame_width(frame_width),
_frame_height(frame_height),
_it(_nh)
{
	_image_pub = _it.advertise("image", 1);

    // configure the camera
    _camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);

}


int AA241xVisionNode::run() {

	// TODO: set up the tag detector
	// TODO: determine if the setup is best in the constructor or here
	// TODO: try another one of the apriltag libraries???

    // open the camera
    std::cout << "Opening Camera..." << std::endl;
	if (!_camera.open()) {
        std::cerr << "Error opening the camera" << std::endl;
        return -1;
    }

	ros::Time image_time; 	// timestamp of when the image was grabbed
	cv::Mat frame_gray;		// the image in grayscale


	// loop while the node should be running
	while (ros::ok()) {

		// TODO: probably want to have a mission state machine running so that
		// this processing is not occuring throughout the entire flight!!!

		// TODO: grab the frame from the camera
        _camera.grab();
		_camera.retrieve(frame_gray);
		image_time = ros::Time::now();


		// TODO: detect the april tags


		// TODO: compute the relative vector between me and the AprilTag
		// NOTE: this will be left entirely to the students I think
		// NOTE: I feel like it would be too easy to just give them that code

		// TODO: publish the detection information


		if (_publish_image) {
			// TODO: add annotation of tag location to the image
			// TODO: publish the annotated image

			std_msgs::Header header;
			header.stamp = image_time;
			sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "mono8", frame_gray).toImageMsg();
			_image_pub.publish(img_msg);
		}


		// TODO: if there are no subscribers, I'm not sure this is needed...
		// though I wonder if it is needed from a time sync point of view
		ros::spinOnce();
	}

    // need to stop the camera
    std::cout << "Stop camera..." << std::endl;
	_camera.release();

}




int main(int argc, char **argv) {

	// initialize th enode
	ros::init(argc, argv, "aa241x_vision_node");

	// get parameters from the launch file which define some mission
	// settings
	ros::NodeHandle private_nh("~");
	// TODO: determine settings
	int frame_width, frame_height;
	bool publish_image;
	private_nh.param("frame_width", frame_width, 640);
	private_nh.param("frame_height", frame_height, 512);
	private_nh.param("publish_image", publish_image, false);

	// create the node
	AA241xVisionNode node(frame_width, frame_height, publish_image);

	// run the node
	return node.run();
}
