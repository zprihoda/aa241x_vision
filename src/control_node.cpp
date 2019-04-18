/**
 * this node will contain a skeleton for students for the control node.
 *
 * this is still in development!
 */


#include <ros/ros.h>



class ControlNode {

public:

	ControlNode();

	int run();


private:


	// node handler
	ros::NodeHandler _nh;

	// TODO: add any settings, etc, here


	// subscribers
	ros::Subscriber _beacon_meas_sub;	// measurement to the beacons (or whatever the meas will be)
	ros::Subscriber _ap_range_sub;		// measurement from the imaging node


	// publishers (?)
	// TODO: set up publishers for data you want to log with rosbag

	// callbacks
	// TODO: add callbacks here
	void beaconMeasCallback(const aa241x_mission::Measurement::ConstPtr& msg);  // NOTE: this might not even be wanted here by students
	void apRangeCallback(const aa241x_student::APRange::ConstPtr& msg);


	// helper functions
	// TODO: define helper functions here



};


ControlNode::ControlNode() {


	// subscribe to the desired topics
	_beacon_meas_sub = _nh.subscribe<aa241x_mission::Measurement>("measurement", 10, &ControlNode::beaconMeasCallback, this);
	_ap_range_sub = _nh.subscribe<aa241x_student::APRange>("ap_range", 10, &ControlNode::apRangeCallback, this);

}

// TODO: I think I am going to remove all beacon measurement related content from this node
// TODO: create another node for the path planning that takes in this information
void ControlNode::beaconMeasCallback(const aa241x_mission::Measurement::ConstPtr& msg) {
	// TODO: decide what to do with the beacon measurement here
}

void ControlNode::apRangeCallback(const aa241x_student::APRange::ConstPtr& msg) {
	// TODO: decide what to do with the range measurement here
	// TODO: basically writing some of the controller in this callback
}



void ControlNode::run() {

	// TODO: copy the code from my research here
	// TODO: should send a 0 motion command until in offboard mode and then
	// should start sending commands as desired to go where desired

	// TODO: maybe path planning should be its own node, but again I'm thinking
	// that is an exercise left to the students
	// this skeleton code should be enough to get them to understand what is
	// going on and how to handle other element (such as path planning)


}

int main(int argc, char **argv) {

	// initialize th enode
	ros::init(argc, argv, "control_node");

	// get parameters from the launch file which define some mission
	// settings
	ros::NodeHandle private_nh("~");
	// TODO: determine settings

	// create the node
	ControlNode node();

	// run the node
	return node.run();
}