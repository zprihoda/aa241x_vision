/**
 * Example node for reading from the Raspberry PiCam and processing it for
 * AprilTag data.
 */

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>
#include <aa241x_vision/tag_info.h>

#include <ros/ros.h>

// raspberry pi cam and image handling
#include <raspicam/raspicam_cv.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// apriltag library
extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>
#include <apriltag/apriltag_pose.h>
}

// topics
#include <geometry_msgs/PoseStamped.h>


/**
 * The node contents in wrapped into a class to allow for easier handling of
 * shared information between the callbacks and main loop
 */
class VisionNode {

public:

	/**
	 * constructor for the vision node
	 * @param frame_width   the integer width of the image frame in pixels
	 * @param frame_height  the integer height of the image frame in pixels
	 * @param publish_image boolean flag for whether or not to publish the images
	 */
	VisionNode(int frame_width, int frame_height, bool publish_image);


	/**
	 * the main loop to be run for this node (called by the `main` function)
	 * @return exit code
	 */
	int run();


private:

	// node handler and image transport handler
	ros::NodeHandle _nh;
	image_transport::ImageTransport _it;


	// settings, etc
	int _frame_width;		// image width to use in [px]
	int _frame_height;		// image height to use in [px]
	bool _publish_image;	// true if the image data should be published

	// setting for apriltag info
	std::map<int, std::vector<std::vector<float>>> _tag_position;
    std::map<int, std::vector<float>> _median_tag_position;

    std::map<int, std::vector<std::vector<float>>> _tag_rotation;
    std::map<int, std::vector<float>> _median_tag_rotation;

    std::vector<int> _tag_numbers{0, 1, 2, 3, 17};

	// camera stuff
	raspicam::RaspiCam_Cv _camera;	// the camera object

	// publishers
	ros::Publisher _tag_info_pub;	// the relative position vector to the truck (NOT IMPLEMENTED)
	ros::Publisher _tag_details_pub;			// the raw tag details (for debugging) (NOT IMPLEMENTED)
	image_transport::Publisher _image_pub;		// the raw annotated image (for debugging)

    void publish_info();
};


VisionNode::VisionNode(int frame_width, int frame_height, bool publish_image) :
_frame_width(frame_width),
_frame_height(frame_height),
_it(_nh)
{
	// publishers
	_image_pub = _it.advertise("image", 1);	// NOTE: should not be used in flight

    // configure the camera
    _camera.set(cv::CAP_PROP_FORMAT, CV_8UC1);				// 8 bit image data -> means grayscale image
    _camera.set(cv::CAP_PROP_FRAME_WIDTH, _frame_width);	// set the width of the image
	_camera.set(cv::CAP_PROP_FRAME_HEIGHT, _frame_height);	// set the height of the image

	// tag information
	_tag_info_pub = _nh.advertise<aa241x_vision::tag_info>("tag_information", 1);
}

void VisionNode::publish_info(){
    aa241x_vision::tag_info msg;
    std::vector<int> _tag_id;
    std::vector<float> _position_all;
    std::vector<float> _position_point;
    std::vector<float> _rotation_all;
    std::vector<float> _rotation_point;
    for(std::map<int, std::vector<float>>::const_iterator it = _median_tag_position.begin(); it != _median_tag_position.end(); it ++){
        _tag_id.push_back(it->first);
        _position_point = it -> second;
        _rotation_point = _median_tag_rotation[it->first];
        for(int j = 0; j < _position_point.size(); j ++){
            _position_all.push_back(_position_point[j]);
        }
        for(int j = 0; j < _rotation_point.size(); j ++){
            _rotation_all.push_back(_rotation_point[j]);
        }
    }
    msg.id = _tag_id;
    msg.position = _position_all;
    msg.rotation = _rotation_all;
    _tag_info_pub.publish(msg);
}


int VisionNode::run() {

    // open the camera
    ROS_INFO("opening camera");
	if (!_camera.open()) {
        ROS_ERROR("Error opening the camera");
        std::cerr << "Error opening the camera" << std::endl;
        return -1;
    }

    // apriltag handling setup
    // see readme for details: https://github.com/AprilRobotics/apriltag
	apriltag_family_t *tf = tag16h5_create();	// specify the tag family

	// initialize the detector and some attributes for detection
	apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 3.0;
    td->quad_sigma = 0.0;
    td->refine_edges = 0;
    //td->decode_sharpening = 0.25;

    double fx, fy, cx, cy;
    if (_frame_width == 1280){
        fx = 1338.9852696569208;
        cx = 640;
        fy = 1338.9852696569208;
        cy = 640;
    }
    else if (_frame_width == 1920){
        fx = 2023.3511151404582;
        cx = 960;
        fy = 2023.3511151404582;
        cy = 960;
    }
    else if (_frame_width == 720){
        fx = 751.35508127802007;
        cx = 360;
        fy = 751.35508127802007;
        cy = 360;
    }
    // collect_state : 0 is not collected, 1 is collected
    int _collect_state = 0;
    double collect_start_time;

	ros::Time image_time; 	// timestamp of when the image was grabbed
	cv::Mat frame_gray;		// the image in grayscale

	// loop forever while the node should be running
	while (ros::ok()) {

		// grab the frame from the camera
		// see readme for details: https://github.com/cedricve/raspicam
        _camera.grab();
		_camera.retrieve(frame_gray);
		image_time = ros::Time::now();

        // Make an image_u8_t header for the Mat data to pass over to the april
        // tag detector
        image_u8_t im = { .width = frame_gray.cols,
            .height = frame_gray.rows,
            .stride = frame_gray.cols,
            .buf = frame_gray.data
        };

        // run the detector
		zarray_t *detections = apriltag_detector_detect(td, &im);
        //ROS_INFO("%d tags detected", zarray_size(detections));

        apriltag_detection_info_t info;
	    info.fx = fx;
	    info.cx = cx;
	    info.fy = fy;
	    info.cy = cy;

        // NOTE: this is where the relative vector should be computed

        // if flagged to do so, annotate and publish the image
		if (_publish_image) {
            // Draw detection outlines
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                info.det = det;
		        if (std::find(_tag_numbers.begin(), _tag_numbers.end(), det->id) == _tag_numbers.end()){continue;}
                if (det->id == 17){info.tagsize = 0.20;}
                else if (det->id == 0 || det->id == 1 || det->id == 2 || det->id == 3){info.tagsize = 0.09;}
		        apriltag_pose_t pose;
		        double err = estimate_tag_pose(&info, &pose);

                // collect position and rotation
                std::vector<float> position{pose.t->data[0], pose.t->data[1], pose.t->data[2]};
                std::vector<float> rotation{pose.R->data[0], pose.R->data[1], pose.R->data[2], pose.R->data[3], pose.R->data[4], pose.R->data[5], pose.R->data[6], pose.R->data[7], pose.R->data[8]};

                _tag_position[det->id].push_back(position);
                _tag_rotation[det->id].push_back(rotation);



                line(frame_gray, cv::Point(det->p[0][0], det->p[0][1]),
                         cv::Point(det->p[1][0], det->p[1][1]),
                         cv::Scalar(0, 0xff, 0), 2);
                line(frame_gray, cv::Point(det->p[0][0], det->p[0][1]),
                         cv::Point(det->p[3][0], det->p[3][1]),
                         cv::Scalar(0, 0, 0xff), 2);
                line(frame_gray, cv::Point(det->p[1][0], det->p[1][1]),
                         cv::Point(det->p[2][0], det->p[2][1]),
                         cv::Scalar(0xff, 0, 0), 2);
                line(frame_gray, cv::Point(det->p[2][0], det->p[2][1]),
                         cv::Point(det->p[3][0], det->p[3][1]),
                         cv::Scalar(0xff, 0, 0), 2);

                std::stringstream ss;
                ss << det->id;
                cv::String text = ss.str();
                int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
                double fontscale = 1.0;
                int baseline;
                cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
                                                &baseline);
                cv::putText(frame_gray, text, cv::Point(det->c[0]-textsize.width/2,
                                           det->c[1]+textsize.height/2),
                        fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
            }

			// publish the annotated image
			std_msgs::Header header;
			header.stamp = image_time;
			sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "mono8", frame_gray).toImageMsg();
			_image_pub.publish(img_msg);
		}

        // clean up the detections
        zarray_destroy(detections);

        //set time threshold for collecting data
        ros::Time time = ros::Time::now();
        if (_collect_state == 0){
            collect_start_time = time.sec;
            _collect_state = 1;
        }

        if (time.sec - collect_start_time > 0.5){
            //extract the most reasonable data
            for (int j = 0 ; j < _tag_numbers.size(); j ++) {
                if (_tag_position[_tag_numbers[j]].size() != 0){
                    //median method
                    std::vector<float> median_position;
                    for(int l = 0 ; l <3; l ++){
                        std::vector<float> x_data;
                        for(int k = 0; k < _tag_position[_tag_numbers[j]].size(); k ++){
                            x_data.push_back(_tag_position[_tag_numbers[j]][k][l]);
                        }
                        sort(x_data.begin(), x_data.end());
                        if (x_data.size() % 2 == 0){
                            median_position.push_back((x_data[x_data.size()/2 - 1] + x_data[x_data.size()/2])/2);
                        }
                        else{
                            median_position.push_back(x_data[x_data.size()/2]);
                        }
                    }
                    _median_tag_position[_tag_numbers[j]] = median_position;

                    std::vector<float> median_rotation;
                    for(int l = 0 ; l <9; l ++){
                        std::vector<float> x_data;
                        for(int k = 0; k < _tag_rotation[_tag_numbers[j]].size(); k ++){
                            x_data.push_back(_tag_rotation[_tag_numbers[j]][k][l]);
                        }
                        sort(x_data.begin(), x_data.end());
                        if (x_data.size() % 2 == 0){
                            median_rotation.push_back((x_data[x_data.size()/2 - 1] + x_data[x_data.size()/2])/2);
                        }
                        else{
                            median_rotation.push_back(x_data[x_data.size()/2]);
                        }
                    }
                    _median_tag_rotation[_tag_numbers[j]] = median_rotation;
                }
            }
            _collect_state = 0;

            publish_info();
            _median_tag_position.clear();
            _tag_position.clear();
            _median_tag_rotation.clear();
            _tag_rotation.clear();
        }

        // call spin once to trigger any callbacks
        // while there are none specified so far, it's good practice to just
        // throw this in so if/when you add callbacks you don't spend hours
        // trying to figure out why they aren't being called!
		ros::spinOnce();
	}

    // need to stop the camera
    ROS_INFO("stopping camera");
	_camera.release();

    // remove apriltag stuff
	apriltag_detector_destroy(td);
	tag16h5_destroy(tf);

}


// the main function


int main(int argc, char **argv) {

	// initialize th enode
	ros::init(argc, argv, "vision_node");

	// get parameters from the launch file which define some camera settings
	ros::NodeHandle private_nh("~");
	int frame_width, frame_height;
	bool publish_image;
	private_nh.param("frame_width", frame_width, 720);
	private_nh.param("frame_height", frame_height, 720);
	private_nh.param("publish_image", publish_image, false);

	// create the node
	VisionNode node(frame_width, frame_height, publish_image);

	// run the node
	return node.run();
}
