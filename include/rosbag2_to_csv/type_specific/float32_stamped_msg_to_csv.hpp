#ifndef TIER4_DEBUG_FLOAT32_STAMPED_MSG_TO_CSV_HPP
#define TIER4_DEBUG_FLOAT32_STAMPED_MSG_TO_CSV_HPP

#include "rosbag2_to_csv/rosbag2_to_csv_base.hpp"

#include "tier4_debug_msgs/msg/float32_stamped.hpp"

class Float32StampedMsgToCsv : public Rosbag2ToCsvBase
{
public:
    Float32StampedMsgToCsv(const std::string &bag_file, const std::string &topic_name, const std::string &csv_file)
        : Rosbag2ToCsvBase(bag_file, topic_name, csv_file) {}

protected:
    std::string convert_msg_to_line(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message) override
    {
        rclcpp::Serialization<tier4_debug_msgs::msg::Float32Stamped> serialization;
        tier4_debug_msgs::msg::Float32Stamped float32_stamped_msg;
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        
        serialization.deserialize_message(&serialized_msg, &float32_stamped_msg);

        std::stringstream ss;
        ss << float32_stamped_msg.stamp.sec << ","
           << float32_stamped_msg.stamp.nanosec << ","
           << std::fixed << std::setprecision(12)
           << float32_stamped_msg.data;

        return ss.str();
    }
};

#endif // TIER4_DEBUG_FLOAT32_STAMPED_MSG_TO_CSV_HPP