#include <memory>
#include <chrono>
#include <cmath>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std;

class ending_search : public rclcpp::Node {
public:
  ending_search() : Node("ending_search") {
    frontiers_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/explore/frontiers", 
      10,
        [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        this->check_frontiers(msg);
    }    );

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 
      10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->check_odom(msg);
    }    );

    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "navigate_to_pose");


    start_time_ = this->get_clock()->now();
    start_time_str_ = generate_timestamp();
    last_odom_time_ = start_time_;
    last_detect_time_ = start_time_;
  }

private:

    rclcpp::Time start_time_;
    std::string start_time_str_;
    rclcpp::Time last_odom_time_;
    rclcpp::Time last_detect_time_;
    geometry_msgs::msg::Point last_odom_pose_;
    int last_frontier_count_ = 0;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr frontiers_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;


    void check_frontiers(const visualization_msgs::msg::MarkerArray::SharedPtr msg){
        int count = msg->markers.size();
        if (count == 0) {
            RCLCPP_INFO(this->get_logger(), "No frontiers detected. Ending search.");
            save_map(start_time_str_ + "_final_map");
            RCLCPP_INFO(this->get_logger(), "Map saved. Shutting down.");
            rclcpp::shutdown();
        } else if (count < 10) {
            if(count <= last_frontier_count_ && (this->get_clock()->now() - last_detect_time_).seconds() > 120.0) {
                RCLCPP_INFO(this->get_logger(), "Few frontiers detected and robot is not moving. Ending search.");
                done_searching();
                rclcpp::shutdown();
            } 
        }
        else {
            last_detect_time_ = this->get_clock()->now();
        }
        RCLCPP_DEBUG(this->get_logger(), "Frontiers detected: %d, last detected: %d", count, last_frontier_count_);
        last_frontier_count_ = count;
    }



    void check_odom(const nav_msgs::msg::Odometry::SharedPtr msg){
        geometry_msgs::msg::Point current_pose = msg->pose.pose.position;
        double distance = sqrt(pow(current_pose.x - last_odom_pose_.x, 2) + pow(current_pose.y - last_odom_pose_.y, 2));
        if (distance < 0.5) { // Robot has not moved significantly, adjust the threshold as needed
            if ((this->get_clock()->now() - last_odom_time_).seconds() > 120.0) {
                RCLCPP_INFO(this->get_logger(), "Robot has not moved for %f seconds. Ending search.", (this->get_clock()->now() - last_odom_time_).seconds());
                done_searching();
                rclcpp::shutdown();
            }
            
        }
        else {
            last_odom_time_ = this->get_clock()->now();
            last_odom_pose_ = current_pose;
        }
        
    }
    




    std::string generate_timestamp() {
        auto now_time = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now_time);
        return std::to_string(time_t_now);
    }

    void save_map(const std::string &filename) {
        const std::string filepath = "/home/jrm/spiader/src/navigation/maps/" + filename;
        std::string cmd = "ros2 run nav2_map_server map_saver_cli -f " + filepath;
        int result = system(cmd.c_str());
        if (result == 0) {
            RCLCPP_INFO(this->get_logger(), "Map saved to %s", filepath.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Map save failed");
        }
    }

    void done_searching(){
        save_map(start_time_str_ + "_final_map");
        RCLCPP_INFO(this->get_logger(), "Map saved.");
        system("pkill -f 'explore'");
        RCLCPP_INFO(this->get_logger(), "Exploration process terminated.");
        //navigate home
        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->get_clock()->now();
        goal.pose.pose.position.x = 0.0;
        goal.pose.pose.position.y = 0.0;
        goal.pose.pose.orientation.w = 1.0;

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
                RCLCPP_INFO(this->get_logger(), "Arrived home. Shutting down.");
                rclcpp::shutdown();
            };

        nav_client_->async_send_goal(goal, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Navigating back to origin...");
    }
};

int main(int argc, char * argv[]) {
  // Initialize ROS 2 communication
  rclcpp::init(argc, argv);
  
  // Spin the node so callbacks are processed
  rclcpp::spin(std::make_shared<ending_search>());
  
  // Shutdown cleanly
  rclcpp::shutdown();
  return 0;
}
