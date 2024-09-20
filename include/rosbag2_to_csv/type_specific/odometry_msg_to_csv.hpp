#ifndef ODOMETRY_MSG_TO_CSV_HPP
#define ODOMETRY_MSG_TO_CSV_HPP

#include "rosbag2_to_csv/rosbag2_to_csv_base.hpp"
#include <nav_msgs/msg/odometry.hpp>

class OdometryMsgToCsv : public Rosbag2ToCsvBase
{
public:
    OdometryMsgToCsv(const std::string &bag_file, const std::string &topic_name, const std::string &csv_file)
        : Rosbag2ToCsvBase(bag_file, topic_name, csv_file) {}

protected:
    std::string convert_msg_to_line(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message) override
    {
        rclcpp::Serialization<nav_msgs::msg::Odometry> serialization;
        nav_msgs::msg::Odometry odometry_msg;
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        
        serialization.deserialize_message(&serialized_msg, &odometry_msg);

        std::stringstream ss;
        ss << odometry_msg.header.stamp.sec << ","
           << odometry_msg.header.stamp.nanosec << ","
           << odometry_msg.header.frame_id << ","
           << odometry_msg.child_frame_id << ","
           << std::fixed << std::setprecision(12)
           << odometry_msg.pose.pose.position.x << ","
           << odometry_msg.pose.pose.position.y << ","
           << odometry_msg.pose.pose.position.z << ","  
           << odometry_msg.pose.pose.orientation.x << ","
           << odometry_msg.pose.pose.orientation.y << ","
           << odometry_msg.pose.pose.orientation.z << ","
           << odometry_msg.pose.pose.orientation.w << ",";

        for (int i = 0; i < 36; i++)
        {
            ss << odometry_msg.pose.covariance[i] << ",";
        }

        ss << odometry_msg.twist.twist.linear.x << ","
           << odometry_msg.twist.twist.linear.y << ","
           << odometry_msg.twist.twist.linear.z << ","
           << odometry_msg.twist.twist.angular.x << ","
           << odometry_msg.twist.twist.angular.y << ","
           << odometry_msg.twist.twist.angular.z << ",";

        for (int i = 0; i < 36; i++)
        {
            ss << odometry_msg.twist.covariance[i];
            if (i < 35)
            {
                ss << ",";
            }
        }

        return ss.str();
    }
};

#endif // ODOMETRY_MSG_TO_CSV_HPP
