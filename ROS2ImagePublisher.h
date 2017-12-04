// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the ROS2IMAGEPUBLISHER_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// ROS2IMAGEPUBLISHER_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef ROS2IMAGEPUBLISHER_EXPORTS
#define ROS2IMAGEPUBLISHER_API __declspec(dllexport)
#else
#define ROS2IMAGEPUBLISHER_API __declspec(dllimport)
#endif

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "custom_msgs/msg/unreal_camera_pose.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/node_impl.hpp"
#include "rclcpp/publisher.hpp"

// This class is exported from the ROS2ImagePublisher.dll
class ROS2IMAGEPUBLISHER_API CROS2ImagePublisher {
public:
	CROS2ImagePublisher(void);
	// TODO: add your methods here.
};

class ROS2IMAGEPUBLISHER_API ROSPublisher {
public:
	bool show_camera = true;
	size_t depth = 10;
	double freq = 30.0;
	rmw_qos_reliability_policy_t reliability_policy;
	rmw_qos_history_policy_t history_policy;
	rmw_qos_profile_t custom_camera_qos_profile;
	rclcpp::node::Node::SharedPtr node; 
	std::shared_ptr<rclcpp::publisher::Publisher<sensor_msgs::msg::Image> > pub;
	//std::shared_ptr<rclcpp::publisher::Publisher<custom_msgs::msg::UnrealCameraPose> > pub;
	//Test
	ROSPublisher(int argc, char * argv[]);

	int PublishTest();

	~ROSPublisher();

	int Publish(cv::Mat mat, bool show_camera);
};

extern ROS2IMAGEPUBLISHER_API int nROS2ImagePublisher;
ROS2IMAGEPUBLISHER_API std::string mat_type2encoding(int mat_type);
ROS2IMAGEPUBLISHER_API void convert_frame_to_message(const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image::SharedPtr msg);

ROS2IMAGEPUBLISHER_API int fnROS2ImagePublisher(void);
ROS2IMAGEPUBLISHER_API int test(int argc, char * argv[]);
