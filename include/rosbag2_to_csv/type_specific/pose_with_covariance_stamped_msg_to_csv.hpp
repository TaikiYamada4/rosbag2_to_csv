#ifndef POSE_WITH_COVARIANCE_STAMPED_MSG_TO_CSV_HPP
#define POSE_WITH_COVARIANCE_STAMPED_MSG_TO_CSV_HPP

#include "rosbag2_to_csv/rosbag2_to_csv_base.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class PoseWithCovarianceStampedMsgToCsv : public Rosbag2ToCsvBase
{
public:
    PoseWithCovarianceStampedMsgToCsv(const std::string &bag_file, const std::string &topic_name, const std::string &csv_file)
        : Rosbag2ToCsvBase(bag_file, topic_name, csv_file) {}

protected:
    std::string convert_msg_to_line(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message) override
    {
        rclcpp::Serialization<geometry_msgs::msg::PoseWithCovarianceStamped> serialization;
        geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_stamped_msg;
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        
        serialization.deserialize_message(&serialized_msg, &pose_with_covariance_stamped_msg);

        std::stringstream ss;
        ss << pose_with_covariance_stamped_msg.header.stamp.sec << ","
           << pose_with_covariance_stamped_msg.header.stamp.nanosec << ","
           << pose_with_covariance_stamped_msg.header.frame_id << ","
           << std::fixed << std::setprecision(12)
           << pose_with_covariance_stamped_msg.pose.pose.position.x << ","
           << pose_with_covariance_stamped_msg.pose.pose.position.y << ","
           << pose_with_covariance_stamped_msg.pose.pose.position.z << ","  
           << pose_with_covariance_stamped_msg.pose.pose.orientation.x << ","
           << pose_with_covariance_stamped_msg.pose.pose.orientation.y << ","
           << pose_with_covariance_stamped_msg.pose.pose.orientation.z << ","
           << pose_with_covariance_stamped_msg.pose.pose.orientation.w << ",";

        for (int i = 0; i < 36; i++)
        {
            ss << pose_with_covariance_stamped_msg.pose.covariance[i];
            if (i < 35)
            {
                ss << ",";
            }
        }

        return ss.str();
    }
};

#endif // POSE_WITH_COVARIANCE_STAMPED_MSG_TO_CSV_HPP