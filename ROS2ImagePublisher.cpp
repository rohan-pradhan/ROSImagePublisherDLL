// ROS2ImagePublisher.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "ROS2ImagePublisher.h"

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

#include "opencv2/highgui/highgui.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

#include "image_tools/options.hpp"

//#include "./burger.hpp"

/// Convert an OpenCV matrix encoding type to a string format recognized by sensor_msgs::Image.
/**
* \param[in] mat_type The OpenCV encoding type.
* \return A string representing the encoding type.
*/
ROS2IMAGEPUBLISHER_API std::string
mat_type2encoding(int mat_type)
{
	switch (mat_type) {
	case CV_8UC1:
		return "mono8";
	case CV_8UC3:
		return "bgr8";
	case CV_16SC1:
		return "mono16";
	case CV_8UC4:
		return "rgba8";
	default:
		throw std::runtime_error("Unsupported encoding type");
	}
}

/// Convert an OpenCV matrix (cv::Mat) to a ROS Image message.
/**
* \param[in] frame The OpenCV matrix/image to convert.
* \param[in] frame_id ID for the ROS message.
* \param[out] Allocated shared pointer for the ROS Image message.
*/
ROS2IMAGEPUBLISHER_API void convert_frame_to_message(
	const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image::SharedPtr msg)
{
	// copy cv information into ros message
	msg->height = frame.rows;
	msg->width = frame.cols;
	msg->encoding = mat_type2encoding(frame.type());
	msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
	size_t size = frame.step * frame.rows;
	msg->data.resize(size);
	memcpy(&msg->data[0], frame.data, size);
	msg->header.frame_id = std::to_string(frame_id);
}

ROSPublisher::ROSPublisher(int argc, char * argv[])
{
	try {
		rclcpp::init(argc, argv);
	}
	/*catch (...)
	{
		printf("won't init");
	}*/
	catch (const std::exception &exc)
	{
		// catch anything thrown within try block that derives from std::exception
		std::cerr << exc.what();
	}
	/*if (rclcpp::ok)
	{
		
	}*/
	show_camera = true;
	depth = 10;
	freq = 30.0;
	reliability_policy = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
	history_policy = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
	std::string topic("image");
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	node = rclcpp::node::Node::make_shared("cam2image");

	rmw_qos_profile_t custom_camera_qos_profile = rmw_qos_profile_default;

	custom_camera_qos_profile.depth = depth;

	custom_camera_qos_profile.reliability = reliability_policy;

	custom_camera_qos_profile.history = history_policy;

	pub = node->create_publisher<sensor_msgs::msg::Image>(
		topic, custom_camera_qos_profile);
	
	return;
}

ROSPublisher::~ROSPublisher()
{
	rclcpp::shutdown();
}

ROS2IMAGEPUBLISHER_API int ROSPublisher::PublishTest()
{
	// Initialize OpenCV image matrices.
	cv::Mat frame;

	// Initialize a shared pointer to an Image message.
	auto msg = std::make_shared<sensor_msgs::msg::Image>();
	msg->is_bigendian = false;

	size_t i = 1;
	cv::Mat mat(100, 100, CV_8UC1);
	double mean = 0.0;
	double stddev = 500.0 / 3.0;

	cv::randn(mat, cv::Scalar(mean), cv::Scalar(stddev));

	frame = mat;
	//}
	// Check if the frame was grabbed correctly
	if (!frame.empty()) {
		// Convert to a ROS image

		convert_frame_to_message(frame, i, msg);

		if (show_camera) {
			// NOTE(esteve): Use C version of cvShowImage to avoid this on Windows:
			// http://stackoverflow.com/questions/20854682/opencv-multiple-unwanted-window-with-garbage-name
			CvMat cvframe = frame;
			// Show the image in a window called "cam2image".
			cvShowImage("cam2image", &cvframe);
			// Draw the image to the screen and wait 1 millisecond.
			cv::waitKey(1);
		}
		// Publish the image message and increment the frame_id.
		printf("Publishing image #%zd\n", i);
		pub->publish(msg);
		++i;
		return 1;
	}
	return 0;
}

ROS2IMAGEPUBLISHER_API int ROSPublisher::Publish(cv::Mat mat, bool show_cam)
{
	// Initialize a shared pointer to an Image message.
	auto msg = std::make_shared<sensor_msgs::msg::Image>();
	msg->is_bigendian = false;
	size_t i = 1;

	if (!mat.empty())
	{
		convert_frame_to_message(mat, i, msg);
		if (show_cam) {
			// NOTE(esteve): Use C version of cvShowImage to avoid this on Windows:
			// http://stackoverflow.com/questions/20854682/opencv-multiple-unwanted-window-with-garbage-name
			CvMat cvframe = mat;
			// Show the image in a window called "cam2image".
			cvShowImage("cam2image", &cvframe);
			// Draw the image to the screen and wait 1 millisecond.
			cv::waitKey(1);
		}
		// Publish the image message and increment the frame_id.
		printf("Publishing image #%zd\n", i);
		pub->publish(msg);
		++i;
		return 1;
	}

	
	return 0;
}


ROS2IMAGEPUBLISHER_API int test(int argc, char * argv[])
{
	// Pass command line arguments to rclcpp.
	rclcpp::init(argc, argv);

	// Initialize default demo parameters
	bool show_camera = true;
	size_t depth = 10;
	double freq = 30.0;
	rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
	rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
	std::string topic("image");

	// Force flush of the stdout buffer.
	// This ensures a correct sync of all prints
	// even when executed simultaneously within a launch file.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	// Initialize a ROS 2 node to publish images read from the OpenCV interface to the camera.
	auto node = rclcpp::node::Node::make_shared("cam2image");

	// Set the parameters of the quality of service profile. Initialize as the default profile
	// and set the QoS parameters specified on the command line.
	rmw_qos_profile_t custom_camera_qos_profile = rmw_qos_profile_default;

	// Depth represents how many messages to store in history when the history policy is KEEP_LAST.
	custom_camera_qos_profile.depth = depth;

	// The reliability policy can be reliable, meaning that the underlying transport layer will try
	// ensure that every message gets received in order, or best effort, meaning that the transport
	// makes no guarantees about the order or reliability of delivery.
	custom_camera_qos_profile.reliability = reliability_policy;

	// The history policy determines how messages are saved until the message is taken by the reader.
	// KEEP_ALL saves all messages until they are taken.
	// KEEP_LAST enforces a limit on the number of messages that are saved, specified by the "depth"
	// parameter.
	custom_camera_qos_profile.history = history_policy;

	printf("Publishing data on topic '%s'\n", topic.c_str());
	// Create the image publisher with our custom QoS profile.
	auto pub = node->create_publisher<sensor_msgs::msg::Image>(
		topic, custom_camera_qos_profile);

	// is_flipped will cause the incoming camera image message to flip about the y-axis.
	bool is_flipped = false;

	// Subscribe to a message that will toggle flipping or not flipping, and manage the state in a
	// callback.
	auto callback =
		[&is_flipped](const std_msgs::msg::Bool::SharedPtr msg) -> void
	{
		is_flipped = msg->data;
		printf("Set flip mode to: %s\n", is_flipped ? "on" : "off");
	};

	// Set the QoS profile for the subscription to the flip message.
	rmw_qos_profile_t custom_flip_qos_profile = rmw_qos_profile_sensor_data;
	custom_flip_qos_profile.depth = 10;

	auto sub = node->create_subscription<std_msgs::msg::Bool>(
		"flip_image", callback, custom_flip_qos_profile);

	// Set a loop rate for our main event loop.
	rclcpp::WallRate loop_rate(freq);

	// Initialize OpenCV image matrices.
	cv::Mat frame;
	cv::Mat flipped_frame;

	// Initialize a shared pointer to an Image message.
	auto msg = std::make_shared<sensor_msgs::msg::Image>();
	msg->is_bigendian = false;

	size_t i = 1;
	cv::Mat mat(100, 100, CV_8UC1);
	double mean = 0.0;
	double stddev = 500.0 / 3.0;

	// Our main event loop will spin until the user presses CTRL-C to exit.
	while (rclcpp::ok()) {
		cv::randn(mat, cv::Scalar(mean), cv::Scalar(stddev));

		//cv::imshow("Display Window", mat);


		// Get the frame from the video capture.
		//if (burger_mode) {
		//	//frame = burger_cap.render_burger(width, height);
		//}
		//else {
		//	//cap >> frame;
		frame = mat;
		//}
		// Check if the frame was grabbed correctly
		if (!frame.empty()) {
			// Convert to a ROS image
			if (!is_flipped) {
				convert_frame_to_message(frame, i, msg);
			}
			else {
				// Flip the frame if needed
				cv::flip(frame, flipped_frame, 1);
				convert_frame_to_message(flipped_frame, i, msg);
			}
			if (show_camera) {
				// NOTE(esteve): Use C version of cvShowImage to avoid this on Windows:
				// http://stackoverflow.com/questions/20854682/opencv-multiple-unwanted-window-with-garbage-name
				CvMat cvframe = frame;
				// Show the image in a window called "cam2image".
				cvShowImage("cam2image", &cvframe);
				// Draw the image to the screen and wait 1 millisecond.
				cv::waitKey(1);
			}
			// Publish the image message and increment the frame_id.
			printf("Publishing image #%zd\n", i);
			pub->publish(msg);
			++i;
		}
		// Do some work in rclcpp and wait for more to come in.
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}

	rclcpp::shutdown();

	return 0;
}



// This is an example of an exported variable
ROS2IMAGEPUBLISHER_API int nROS2ImagePublisher=0;

// This is an example of an exported function.
ROS2IMAGEPUBLISHER_API int fnROS2ImagePublisher(void)
{
    return 42;
}

// This is the constructor of a class that has been exported.
// see ROS2ImagePublisher.h for the class definition
CROS2ImagePublisher::CROS2ImagePublisher()
{
    return;
}
