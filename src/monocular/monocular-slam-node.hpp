#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include <opencv2/opencv.hpp>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);
    ~MonocularSlamNode();

private:
    void GrabImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr m_image_subscriber;
};

#endif
