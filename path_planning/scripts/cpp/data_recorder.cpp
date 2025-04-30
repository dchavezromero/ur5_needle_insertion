// imports
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <condition_variable>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <builtin_interfaces/msg/time.hpp>

// Global variables for joint state recording
std::mutex joint_state_mutex;
std::condition_variable joint_state_cv;
bool recording = false;
bool recording_complete = false;
std::vector<sensor_msgs::msg::JointState> recorded_states;
builtin_interfaces::msg::Time recording_start_time;
bool start_time_initialized = false;  // Flag to track if start time has been set

// Get current timestamp for filename
std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
    return ss.str();
}

// Function to save trajectory data to CSV
void saveJointStatesToCSV(const std::vector<sensor_msgs::msg::JointState>& joint_states,
                          const std::string& prefix,
                          const rclcpp::Logger& logger) {
    if (joint_states.empty()) {
        RCLCPP_WARN(logger, "No joint states recorded, skipping CSV generation");
        return;
    }

    std::string timestamp = getCurrentTimestamp();
    std::string filename = prefix + "_" + timestamp + ".csv";
    
    std::ofstream csv_file(filename);
    if (!csv_file.is_open()) {
        RCLCPP_ERROR(logger, "Failed to open file for writing: %s", filename.c_str());
        return;
    }

    // List of joints we're interested in
    const std::vector<std::string> joint_names = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };

    // Write header
    csv_file << "time,";
    for (size_t i = 0; i < joint_names.size(); ++i) {
        csv_file << joint_names[i];
        if (i < joint_names.size() - 1) csv_file << ",";
    }
    csv_file << "\n";

    // Get indices of the joints we're interested in for the first message
    std::vector<size_t> joint_indices;
    const auto& first_state = joint_states[0];
    for (const auto& joint_name : joint_names) {
        auto it = std::find(first_state.name.begin(), first_state.name.end(), joint_name);
        if (it != first_state.name.end()) {
            joint_indices.push_back(std::distance(first_state.name.begin(), it));
        } else {
            RCLCPP_ERROR(logger, "Joint %s not found in recorded joint states", joint_name.c_str());
            return;
        }
    }

    // Get the start time from the first message
    const auto& first_msg = joint_states[0];
    double start_sec = first_msg.header.stamp.sec;
    double start_nsec = first_msg.header.stamp.nanosec;
    
    // Write joint states
    for (const auto& state : joint_states) {
        // Calculate time relative to the first message in seconds
        double time_secs = (state.header.stamp.sec - start_sec) + 
                          ((state.header.stamp.nanosec - start_nsec) * 1e-9);
        
        // Handle negative time that might occur due to precision errors
        if (time_secs < 0) time_secs = 0.0;
        
        csv_file << time_secs << ",";
        
        // Write joint positions
        for (size_t i = 0; i < joint_indices.size(); ++i) {
            size_t idx = joint_indices[i];
            if (idx < state.position.size()) {
                csv_file << state.position[idx];
            } else {
                csv_file << "NaN"; // Handle missing data
                RCLCPP_WARN(logger, "Missing position data for joint at index %zu", idx);
            }
            
            if (i < joint_indices.size() - 1) csv_file << ",";
        }
        csv_file << "\n";
    }

    RCLCPP_INFO(logger, "Saved %zu joint states to %s", joint_states.size(), filename.c_str());
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "data_recorder", 
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true)
            .use_intra_process_comms(true)
            .append_parameter_override("use_sim_time", true));
    
    auto logger = node->get_logger();
    
    // Check if use_sim_time is correctly set
    bool use_sim_time = false;
    node->get_parameter("use_sim_time", use_sim_time);
    RCLCPP_INFO(logger, "Data Recorder Node Initialized with use_sim_time=%s", 
               use_sim_time ? "true" : "false");

    // Joint state subscriber for recording
    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [&logger](const sensor_msgs::msg::JointState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(joint_state_mutex);
            if (recording && !recording_complete) {
                recorded_states.push_back(*msg);
                
                if (recorded_states.size() % 100 == 0) {
                    RCLCPP_INFO(logger, "Recorded %zu joint states", recorded_states.size());
                }
            }
        });

    // Create service to start recording
    auto start_recording_service = node->create_service<std_srvs::srv::Trigger>(
        "start_recording",
        [&logger](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            {
                std::lock_guard<std::mutex> lock(joint_state_mutex);
                recorded_states.clear();
                recording = true;
                recording_complete = false;
            }
            RCLCPP_INFO(logger, "Started recording joint states");
            response->success = true;
            response->message = "Recording started";
            return;
        });

    // Create service to stop recording and save data
    auto stop_recording_service = node->create_service<std_srvs::srv::Trigger>(
        "stop_recording",
        [&logger](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            std::vector<sensor_msgs::msg::JointState> states_copy;
            {
                std::lock_guard<std::mutex> lock(joint_state_mutex);
                recording_complete = true;
                recording = false;
                states_copy = recorded_states;
            }
            
            // Save the recorded data
            saveJointStatesToCSV(states_copy, "joint_trajectory", logger);
            
            RCLCPP_INFO(logger, "Stopped recording and saved %zu joint states", states_copy.size());
            response->success = true;
            response->message = "Recording stopped and data saved";
            return;
        });

    RCLCPP_INFO(logger, "Data recorder ready. Use services to start/stop recording.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
