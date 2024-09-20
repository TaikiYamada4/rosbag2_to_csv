#ifndef ROSBAG2_TO_CSV_BASE_HPP
#define ROSBAG2_TO_CSV_BASE_HPP

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <fstream>
#include <iomanip>

class Rosbag2ToCsvBase
{
public:
    Rosbag2ToCsvBase(const std::string &bag_file, const std::string &topic_name, const std::string &csv_file)
        : bag_file_(bag_file), topic_name_(topic_name), csv_file_(csv_file) {}

    virtual ~Rosbag2ToCsvBase() {}

    void process_bag()
    {
        rosbag2_cpp::readers::SequentialReader reader;
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_file_;
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        reader.open(storage_options, converter_options);

        std::vector<std::string> csv_data;

        while (reader.has_next())
        {
            auto bag_message = reader.read_next();
            if (bag_message->topic_name == topic_name_)
            {
                std::string csv_line = convert_msg_to_line(bag_message);
                csv_data.push_back(csv_line);
            }
        }

        write_lines_to_csv(csv_file_, csv_data);
    }

protected:
    virtual std::string convert_msg_to_line(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message) = 0;

private:
    void write_lines_to_csv(const std::string &filename, const std::vector<std::string> &data)
    {
        std::ofstream file(filename, std::ios::out);

        for (const auto &line : data)
        {
            file << line << "\n";
        }
        file.close();
    }

    std::string bag_file_;
    std::string topic_name_;
    std::string csv_file_;
};


#endif // ROSBAG2_TO_CSV_BASE_HPP