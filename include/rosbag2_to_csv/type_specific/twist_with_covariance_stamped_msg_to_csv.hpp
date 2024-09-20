#ifndef TWIST_WITH_COVARIANCE_STAMPED_MSG_TO_CSV_HPP
#define TWIST_WITH_COVARIANCE_STAMPED_MSG_TO_CSV_HPP

#include "rosbag2_to_csv/rosbag2_to_csv_base.hpp"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

class TwistWithCovarianceStampedMsgToCsv : public Rosbag2ToCsvBase
{
public:
    TwistWithCovarianceStampedMsgToCsv(const std::string &bag_file, const std::string &topic_name, const std::string &csv_file)
        : Rosbag2ToCsvBase(bag_file, topic_name, csv_file) {}

protected:
    std::string convert_msg_to_line(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message) override
    {
        rclcpp::Serialization<geometry_msgs::msg::TwistWithCovarianceStamped> serialization;
        geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance_stamped_msg;
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        
        serialization.deserialize_message(&serialized_msg, &twist_with_covariance_stamped_msg);

        std::stringstream ss;
        ss << twist_with_covariance_stamped_msg.header.stamp.sec << ","
           << twist_with_covariance_stamped_msg.header.stamp.nanosec << ","
           << twist_with_covariance_stamped_msg.header.frame_id << ","
           << std::fixed << std::setprecision(12)
           << twist_with_covariance_stamped_msg.twist.twist.linear.x << ","
           << twist_with_covariance_stamped_msg.twist.twist.linear.y << ","
           << twist_with_covariance_stamped_msg.twist.twist.linear.z << ","  
           << twist_with_covariance_stamped_msg.twist.twist.angular.x << ","
           << twist_with_covariance_stamped_msg.twist.twist.angular.y << ","
           << twist_with_covariance_stamped_msg.twist.twist.angular.z << ",";

        for (int i = 0; i < 36; i++)
        {
            ss << twist_with_covariance_stamped_msg.twist.covariance[i];
            if (i < 35)
            {
                ss << ",";
            }
        }

        return ss.str();
    }
};

#endif // TWIST_WITH_COVARIANCE_STAMPED_MSG_TO_CSV_HPP