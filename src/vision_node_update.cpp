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
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>

#include <ros/ros.h>
#include <aa241x_student/tag_info.h>

#include <raspicam/raspicam_cv.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>
#include <apriltag/apriltag_pose.h>
}



// TODO: remove aa241x from the node name
class VisionNode {

public:

	VisionNode(int frame_width, int frame_height, bool publish_image);


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
    std::map<int, std::vector<std::vector<float>>> _tag_position;
    std::map<int, std::vector<float>> _median_tag_position;

    std::map<int, std::vector<std::vector<float>>> _tag_rotation;
    std::map<int, std::vector<float>> _median_tag_rotation;

    std::vector<int> _tag_numbers{0, 3};

	// subscribers
	// TODO: figure out what subscriber may be desired


	// publishers
	ros::Publisher _tag_info_pub;	// the tag info
	ros::Publisher _tag_details_pub;			// the raw tag details (for debugging)
	image_transport::Publisher _image_pub;

	// TODO: add a publisher of the image frame -> for the display node

	// callbacks
	// TODO: figure out what might be needed


	// helper functions
	// TODO: any helper functions here

    // publish the translation and rotation matrix
    void publish_info();

};


VisionNode::VisionNode(int frame_width, int frame_height, bool publish_image) :
_frame_width(frame_width),
_frame_height(frame_height),
_it(_nh)
{
	_image_pub = _it.advertise("image", 1);

    // configure the camera
    _camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
    _camera.set(cv::CAP_PROP_FRAME_WIDTH, _frame_width);
	_camera.set(cv::CAP_PROP_FRAME_HEIGHT, _frame_height);
	_camera.set(cv::CAP_PROP_FORMAT, CV_8UC1);

    _tag_info_pub = _nh.advertise<aa241x_student::tag_info>("tag_information", 5);
}


void VisionNode::publish_info(){
    aa241x_student::tag_info msg;
    std::vector<int> _tag_id;
    std::vector<float> _position_all;
    std::vector<float> _position_point;
    std::vector<float> _rotation_all;
    std::vector<float> _rotation_point;
    for(std::map<int, std::vector<float>>::const_iterator it = _median_tag_position.begin(); it != _median_tag_position.end(); it ++){
        _tag_id.push_back(it->first);
        _position_point = it -> second;
        for(int j = 0; j < _position_point.size(); j ++){
            _position_all.push_back(_position_point[j]);
        }
        _rotation_point = _median_tag_rotation[it->first];
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

	// TODO: set up the tag detector
	// TODO: determine if the setup is best in the constructor or here
	// TODO: try another one of the apriltag libraries???

    // open the camera
    ROS_INFO("opening camera");
	if (!_camera.open()) {
        ROS_ERROR("Error opening the camera");
        std::cerr << "Error opening the camera" << std::endl;
        return -1;
    }

    // apriltag handling setup
	apriltag_family_t *tf = tag16h5_create();

	apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 3.0;
    td->quad_sigma = 0.0;
    td->refine_edges = 0;
    //td->decode_sharpening = 0.25;

	ros::Time image_time; 	// timestamp of when the image was grabbed
	cv::Mat frame_gray;		// the image in grayscale

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

    int _collect_state = 0;
    double collect_start_time;

    /*
    std::map<int, std::vector<std::vector<float>>> _tag_position;
    std::map<int, std::vector<int>> _tag_count;
    std::map<int, std::vector<float>> _final_tag_position;*/



	// loop while the node should be running
	while (ros::ok()) {

		// TODO: probably want to have a mission state machine running so that
		// this processing is not occuring throughout the entire flight!!!

		// TODO: grab the frame from the camera
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

		zarray_t *detections = apriltag_detector_detect(td, &im);
        //ROS_INFO("%d tags detected", zarray_size(detections));

	    apriltag_detection_info_t info;
	    info.tagsize = 0.09;
	    info.fx = fx;
	    info.cx = cx;
	    info.fy = fy;
	    info.cy = cy;

        //apriltag_detection_info_t info_small;
	    //info.tagsize = 0.16;
	    //info.fx = fx;
	    //info.cx = cx;
	    //info.fy = fy;
	    //info.cy = cy;



		// TODO: compute the relative vector between me and the AprilTag
		// NOTE: this will be left entirely to the students I think
		// NOTE: I feel like it would be too easy to just give them that code

		// TODO: publish the detection information


		if (_publish_image) {

            // Draw detection outlines
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                //To Do: filtering algorithm
		        info.det = det;
		        if (std::find(_tag_numbers.begin(), _tag_numbers.end(), det->id) == _tag_numbers.end()){continue;}

		        apriltag_pose_t pose;
		        double err = estimate_tag_pose(&info, &pose);

                /*float determinant = pose.R->data[0] * pose.R->data[4] * pose.R->data[8] + pose.R->data[1] * pose.R->data[5] * pose.R->data[6] + pose.R->data[2] * pose.R->data[3] * pose.R->data[7] - (pose.R->data[1] * pose.R->data[3] * pose.R->data[8] + pose.R->data[2] * pose.R->data[4] * pose.R->data[6] + pose.R->data[0] * pose.R->data[5] * pose.R->data[7]);
                float inv_R_0 = pose.R->data[4] * pose.R->data[8] - pose.R->data[5] * pose.R->data[7];
                float inv_R_1 = pose.R->data[2] * pose.R->data[7] - pose.R->data[1] * pose.R->data[8];
                float inv_R_2 = pose.R->data[1] * pose.R->data[5] - pose.R->data[2] * pose.R->data[4];
                float inv_R_3 = pose.R->data[5] * pose.R->data[6] - pose.R->data[3] * pose.R->data[8];
                float inv_R_4 = pose.R->data[0] * pose.R->data[8] - pose.R->data[2] * pose.R->data[6];
                float inv_R_5 = pose.R->data[2] * pose.R->data[3] - pose.R->data[0] * pose.R->data[5];
                float inv_R_6 = pose.R->data[3] * pose.R->data[7] - pose.R->data[4] * pose.R->data[6];
                float inv_R_7 = pose.R->data[1] * pose.R->data[6] - pose.R->data[0] * pose.R->data[7];
                float inv_R_8 = pose.R->data[0] * pose.R->data[4] - pose.R->data[1] * pose.R->data[3];

                // Translation vector from apriltag to camera frame
                float trans_x = (inv_R_0 * pose.t->data[0] + inv_R_1 * pose.t->data[1] + inv_R_2 * pose.t->data[2]) / determinant;
                float trans_y = (inv_R_3 * pose.t->data[0] + inv_R_4 * pose.t->data[1] + inv_R_5 * pose.t->data[2]) / determinant;
                float trans_z = (inv_R_6 * pose.t->data[0] + inv_R_7 * pose.t->data[1] + inv_R_8 * pose.t->data[2]) / determinant;
                */

                std::vector<float> position{pose.t->data[0], pose.t->data[1], pose.t->data[2]};
                std::vector<float> rotation{pose.R->data[0], pose.R->data[1], pose.R->data[2], pose.R->data[3], pose.R->data[4], pose.R->data[5], pose.R->data[6], pose.R->data[7], pose.R->data[8]};
                //ROS_INFO("!!!!!!!!!!!!@@##$$%%^^^^^$#@@");
                //ROS_INFO("%d %.3f %.3f %.3f",det->id,trans_x,trans_y, trans_z);

                /*
                //clustering data by distance threshold
                std::vector<float> prev_position;
                float distance;
                bool update;
                if (_tag_position[det->id].size() == 0){
                    _tag_position[det->id].push_back(position);
                    _tag_count[det->id].push_back(1);
                }
                else{
                    update = false;
                    for (int i = 0 ; i < _tag_position[det->id].size(); i ++){

                        prev_position = _tag_position[det->id][i];
                        distance = std::sqrt(std::pow(prev_position[0] - position[0], 2.0) + std::pow(prev_position[1] - position[1], 2.0) + std::pow(prev_position[2] - position[2], 2.0));

                        if (distance < 0.1){
                            int prev_count = _tag_count[det->id][i];

                            std::vector<float> update_position;
                            for (int j = 0; j < 3; j ++){
                                update_position.push_back((prev_position[j] * prev_count + position[j])/(prev_count + 1));
                            }

                            _tag_position[det->id][i] = update_position;
                            _tag_count[det->id][i] = prev_count + 1;

                            update = true;
                            break;
                        }
                    }
                    if(update != true){
                        _tag_position[det->id].push_back(position);
                        _tag_count[det->id].push_back(1);
                    }
                }*/

                //collect all data
                _tag_position[det->id].push_back(position);
                _tag_rotation[det->id].push_back(rotation);

		        //ROS_INFO("%d %.3f %.3f %.3f",det->id,trans_x,trans_y, trans_z);

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

        if (time.sec - collect_start_time > 5){
            //extract the most reasonable data
            for (int j = 0 ; j < _tag_numbers.size(); j ++) {
                if (_tag_position[_tag_numbers[j]].size() != 0){
                    /*//clustering method
                    std::vector<int> count_vector = _tag_count[_tag_numbers[j]];
                    int _max_index = std::distance(count_vector.begin(), std::max_element(count_vector.begin(), count_vector.end()));
                    _final_tag_position[_tag_numbers[j]] = _tag_position[_tag_numbers[j]][_max_index];*/

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

            //check the tag formation
            //float format_distance = std::sqrt(std::pow((_final_tag_position[0][0] - _final_tag_position[3][0]), 2.0) + std::pow((_final_tag_position[0][1] - _final_tag_position[3][1]), 2.0) + std::pow((_final_tag_position[0][2] - _final_tag_position[3][2]), 2.0));
            //format_distance = 0.14;

            if (true){
                _collect_state = 0;
                ROS_INFO("Found resonable tags");
                /*
                ROS_INFO("%d %d", _tag_position[0].size(), _tag_position[3].size());
                _tag_position.clear();
                _tag_count.clear();
                ROS_INFO("Average of cluster");
                ROS_INFO("%d %.3f %.3f %.3f",_tag_numbers[0],_final_tag_position[0][0],_final_tag_position[0][1], _final_tag_position[0][2]);
                ROS_INFO("%d %.3f %.3f %.3f",_tag_numbers[1],_final_tag_position[3][0],_final_tag_position[3][1], _final_tag_position[3][2]);
                _final_tag_position.clear();*/
                for(int j = 0; j < _tag_numbers.size(); j ++){
                    if(_tag_position[_tag_numbers[j]].size() != 0){
                    ROS_INFO("Median of all");
                    ROS_INFO("%d %.3f %.3f %.3f",_tag_numbers[j],_median_tag_position[_tag_numbers[j]][0],_median_tag_position[_tag_numbers[j]][1], _median_tag_position[_tag_numbers[j]][2]);

                    ROS_INFO("%.3f %.3f %.3f",_median_tag_rotation[_tag_numbers[j]][0],_median_tag_rotation[_tag_numbers[j]][1], _median_tag_rotation[_tag_numbers[j]][2]);
                    ROS_INFO("%.3f %.3f %.3f",_median_tag_rotation[_tag_numbers[j]][3],_median_tag_rotation[_tag_numbers[j]][4], _median_tag_rotation[_tag_numbers[j]][5]);
                    ROS_INFO("%.3f %.3f %.3f",_median_tag_rotation[_tag_numbers[j]][6],_median_tag_rotation[_tag_numbers[j]][7], _median_tag_rotation[_tag_numbers[j]][8]);
                    }
                }
                publish_info();
                _median_tag_position.clear();
                _tag_position.clear();
                _median_tag_rotation.clear();
                _tag_rotation.clear();
            }
        }

		// TODO: if there are no subscribers, I'm not sure this is needed...
		// though I wonder if it is needed from a time sync point of view
		ros::spinOnce();
	}

    // need to stop the camera
    ROS_INFO("stopping camera");
	_camera.release();

    // remove apriltag stuff
	apriltag_detector_destroy(td);
	tag16h5_destroy(tf);

}




int main(int argc, char **argv) {

	// initialize th enode
	ros::init(argc, argv, "vision_node");

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
	VisionNode node(frame_width, frame_height, publish_image);

	// run the node
	return node.run();
}
