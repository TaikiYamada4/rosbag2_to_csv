#include "rosbag2_to_csv/rosbag2_to_csv_base.hpp"
#include "rosbag2_to_csv/type_specific/pose_with_covariance_stamped_msg_to_csv.hpp"
#include "rosbag2_to_csv/type_specific/twist_with_covariance_stamped_msg_to_csv.hpp"
#include "rosbag2_to_csv/type_specific/imu_msg_to_csv.hpp"
#include "rosbag2_to_csv/type_specific/odometry_msg_to_csv.hpp"
#include "rosbag2_to_csv/type_specific/float32_stamped_msg_to_csv.hpp"

int main(int argc, char **argv) 
{
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " <bag_file> <topic_name> <message_type> <csv_file>" << std::endl;
        return 1;
    }

    std::string bag_file = argv[1];
    std::string topic_name = argv[2];
    std::string message_type = argv[3];
    std::string csv_file = argv[4];

    rclcpp::init(argc, argv);

    std::unique_ptr<Rosbag2ToCsvBase> converter;

    if(message_type == "geometry_msgs/msg/PoseWithCovarianceStamped") {
        converter = std::make_unique<PoseWithCovarianceStampedMsgToCsv>(bag_file, topic_name, csv_file);
    }
    else if(message_type == "geometry_msgs/msg/TwistWithCovarianceStamped") {
        converter = std::make_unique<TwistWithCovarianceStampedMsgToCsv>(bag_file, topic_name, csv_file);
    } 
    else if(message_type == "sensor_msgs/msg/Imu") {
        converter = std::make_unique<ImuMsgToCsv>(bag_file, topic_name, csv_file);
    }
    else if(message_type == "nav_msgs/msg/Odometry"){
        converter = std::make_unique<OdometryMsgToCsv>(bag_file, topic_name, csv_file);
    }
    else if(message_type == "tier4_debug_msgs/msg/Float32Stamped") {
        converter = std::make_unique<Float32StampedMsgToCsv>(bag_file, topic_name, csv_file);
    }
    else {
        std::cerr << "Unsupported message type: " << message_type << std::endl;
        return 1;
    }

    try {
        converter->process_bag();
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}