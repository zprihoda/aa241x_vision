# AA241x Student Example Code #

This ROS package contains some example nodes (and code) for the teams of Spring 2019's AA241x course.

### Dependencies ###

This code has the following dependencies:

 - [RaspiCam C++](https://github.com/cedricve/raspicam) - this is a library that enables interfacing with the PiCam using C++.

 - [APrilTag Library](https://github.com/AprilRobotics/apriltag) - this is the original AprilTag library in C put out by the April Robotics Lab.  **This is not the only library out there and may not be the best performance library.**


**NOTE:** these dependencies should already be installed on the Raspberry Pi 3B+ received by the teams.

### ROS ###

For a more detailed explanation of ROS, the elements of a package, and some of the code elements, check out the [ROS Tutorials page](http://wiki.ros.org/ROS/Tutorials) that contains a lot of very useful information (tutorials 2-6, 8, 10, and 11 should be read).  In addition to the tutorials, an effort has been made to comment the code to help explain what some of the elements are doing.

## Nodes ##

This package contains the following nodes (see each of the corresponding sections for a little more detail on the nodes):

 - [Vision Node](#vision-node) - this node is responsible for reading in the images from the PiCam and passing the images through an APrilTag detector.

 - [Display Node](#display-node) - this node is responsible for displaying the annotated images from the vision node.

 - [Control Node](#control-node) - this node is responsible for sending control commands to the pixhawk autopilot through mavros.

**NOTE:** nodes can have parameters that define or alter some of the functionality when the node is run, which is being leveraged in the vision and display nodes for adjusting the size of the image being shown.  For more details on node parameters, check out [this ROS help page on Nodes](http://wiki.ros.org/Nodes).


### Vision Node ###

The vision node does the heavy lifting of reading an image from the camera and running the image through the AprilTag detector.  Depending on the input flags, those images, annotated with the AprilTag information will be published so the (Display Node)[#display-node] can display the images.

Parameters for the node:
 - `frame_width`: integer number of pixels for the width of the image frame
 - `frame_height`: integer number of pixels of the height of the image frame
 - `display_video`: a boolean flag to determine whether or not to publish video frames for the display node to display

**NOTE:** If the `display_video` parameter is `false`, images will never display, regardless of whether or not the display node is running!

### Display Node ###

Displays the images using an OpenCV image frame.

Parameters for the node:
 - `frame_width`: integer number of pixels for the width of the image frame
 - `frame_height`: integer number of pixels of the height of the image frame

