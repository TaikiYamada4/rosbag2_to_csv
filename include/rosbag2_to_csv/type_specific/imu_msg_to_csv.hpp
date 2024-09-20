#ifndef IMU_MSG_TO_CSV_HPP
#define IMU_MSG_TO_CSV_HPP

#include "rosbag2_to_csv/rosbag2_to_csv_base.hpp"
#include <sensor_msgs/msg/imu.hpp>

class ImuMsgToCsv : public Rosbag2ToCsvBase
{
public:
    ImuMsgToCsv(const std::string &bag_file, const std::string &topic_name, const std::string &csv_file)
        : Rosbag2ToCsvBase(bag_file, topic_name, csv_file) {}

protected:
    std::string convert_msg_to_line(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message) override
    {
        rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
        sensor_msgs::msg::Imu imu_msg;
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        
        serialization.deserialize_message(&serialized_msg, &imu_msg);

        std::stringstream ss;
        ss  << imu_msg.header.stamp.sec << ","
            << imu_msg.header.stamp.nanosec << ","
            << imu_msg.header.frame_id << ","
            << std::fixed << std::setprecision(12)
            << imu_msg.orientation.x << ","
            << imu_msg.orientation.y << ","
            << imu_msg.orientation.z << ","
            << imu_msg.orientation.w << ","
            << imu_msg.orientation_covariance[0] << ","
            << imu_msg.orientation_covariance[1] << ","
            << imu_msg.orientation_covariance[2] << ","
            << imu_msg.orientation_covariance[3] << ","
            << imu_msg.orientation_covariance[4] << ","
            << imu_msg.orientation_covariance[5] << ","
            << imu_msg.orientation_covariance[6] << ","
            << imu_msg.orientation_covariance[7] << ","
            << imu_msg.orientation_covariance[8] << ","
            << imu_msg.angular_velocity.x << ","
            << imu_msg.angular_velocity.y << ","
            << imu_msg.angular_velocity.z << ","
            << imu_msg.angular_velocity_covariance[0] << ","
            << imu_msg.angular_velocity_covariance[1] << ","
            << imu_msg.angular_velocity_covariance[2] << ","
            << imu_msg.angular_velocity_covariance[3] << ","
            << imu_msg.angular_velocity_covariance[4] << ","
            << imu_msg.angular_velocity_covariance[5] << ","
            << imu_msg.angular_velocity_covariance[6] << ","
            << imu_msg.angular_velocity_covariance[7] << ","
            << imu_msg.angular_velocity_covariance[8] << ","
            << imu_msg.linear_acceleration.x << ","
            << imu_msg.linear_acceleration.y << ","
            << imu_msg.linear_acceleration.z << ","
            << imu_msg.linear_acceleration_covariance[0] << ","
            << imu_msg.linear_acceleration_covariance[1] << ","
            << imu_msg.linear_acceleration_covariance[2] << ","
            << imu_msg.linear_acceleration_covariance[3] << ","
            << imu_msg.linear_acceleration_covariance[4] << ","
            << imu_msg.linear_acceleration_covariance[5] << ","
            << imu_msg.linear_acceleration_covariance[6] << ","
            << imu_msg.linear_acceleration_covariance[7] << ","
            << imu_msg.linear_acceleration_covariance[8];

        return ss.str();
    }
};

#endif // IMU_MSG_TO_CSV_HPP