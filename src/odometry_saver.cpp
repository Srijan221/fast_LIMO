#include <fstream>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>


class OdometrySaver : public rclcpp::Node {
public:
    OdometrySaver() : Node("odometry_saver") {
        // Create output file name with timestamp
        output_filename_ = makeTimestampedFilename();

        // Create the output file if it doesnt exist
        initializeOutputFile();
        
        // Subscribe to state topic from fast limo
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/fast_limo/state", 10, std::bind(&OdometrySaver::odometryCallback, this, std::placeholders::_1));
    }

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto& pose = msg->pose.pose;
        
        // Open the file
        std::ofstream file(output_filename_, std::ios::app); 
        //this is append mode: std::ios::app
        //default is: std::ios::out - which overwrites

        // Append contents to csv file
        if (file.is_open()) {
            // Use the seconds and nsecs to create timestamp
            double stamp = rclcpp::Time(msg->header.stamp).seconds();

            
            file << std::fixed << std::setprecision(9)
                << stamp << ","
                << pose.position.x << ","
                << pose.position.y << ","
                << pose.position.z << ","
                << pose.orientation.x << ","
                << pose.orientation.y << ","
                << pose.orientation.z << ","
                << pose.orientation.w << "\n";
            file.close();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", output_filename_.c_str());
        }
    }

    void initializeOutputFile() {
        // Initialize the file if it doesnt exist already
        if (!std::filesystem::exists(output_filename_)) {
            std::ofstream header_file(output_filename_, std::ios::out);
            if (header_file.is_open()) {
                // Add the header line of the csv
                header_file << "timestamp,tx,ty,tz,qx,qy,qz,qw\n";
                header_file.close();
                RCLCPP_INFO(this->get_logger(), "Created new odometry CSV file: %s", output_filename_.c_str());
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Appending to existing odometry CSV file: %s", output_filename_.c_str());
        }
    }

    std::string makeTimestampedFilename() {
        // Get time
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm;
        localtime_r(&t, &tm);

        // Create filename
        char buf[64];
        strftime(buf, sizeof(buf), "odometry_%Y-%m-%d_%H-%M-%S.csv", &tm);

        return std::string(buf);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::string output_filename_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometrySaver>());
    rclcpp::shutdown();
    return 0;
}
