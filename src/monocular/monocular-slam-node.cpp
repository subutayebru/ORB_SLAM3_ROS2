#include "monocular-slam-node.hpp"
#include <opencv2/imgcodecs.hpp>

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
    : Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;

    // Create QoS profile to match the bag file
    auto qos = rclcpp::QoS(rclcpp::KeepLast(3))
        .best_effort()
        .durability_volatile();

    m_image_subscriber = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/uuv02/vertical_camera/image_raw/compressed",
        qos,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "SLAM node initialized");
}

MonocularSlamNode::~MonocularSlamNode()
{
    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    try
    {
        // Decode the compressed image
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_UNCHANGED);
        
        if (image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
            return;
        }

        // Get the timestamp in seconds
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        RCLCPP_INFO(this->get_logger(), "Processing frame at timestamp: %f", timestamp);

        // Process the frame with ORB-SLAM3
        m_SLAM->TrackMonocular(image, timestamp);

        RCLCPP_INFO(this->get_logger(), "Frame processed successfully");
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception in GrabImage: %s", e.what());
    }
}
