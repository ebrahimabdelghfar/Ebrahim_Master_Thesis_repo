#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <memory>
#include <functional>
#include <vector>
#include <string>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/precompute.hpp"

using namespace racecar_simulator;
using std::placeholders::_1;

class BehaviorController : public rclcpp::Node {
private:
    // Listen for messages from joystick, keyboard, laser scan, odometry, and IMU
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr brake_bool_sub;

    // Publisher for mux controller
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr mux_pub;

    // Mux indices
    int joy_mux_idx;
    int key_mux_idx;
    int random_walker_mux_idx;
    int nav_mux_idx;
    int brake_mux_idx;

    // Mux controller array
    std::vector<bool> mux_controller;
    int mux_size;

    // Button indices
    int joy_button_idx;
    int key_button_idx;
    int random_walk_button_idx;
    int brake_button_idx;
    int nav_button_idx;

    // Key indices
    std::string joy_key_char;
    std::string keyboard_key_char;
    std::string brake_key_char;
    std::string random_walk_key_char;
    std::string nav_key_char;

    // Is ebrake on? (not engaged, but on)
    bool safety_on;

    // To roughly keep track of vehicle state
    racecar_simulator::CarState state;

    // precompute distance from lidar to edge of car for each beam
    std::vector<double> car_distances;

    // precompute cosines of scan angles
    std::vector<double> cosines;

    // for collision detection
    double ttc_threshold;
    bool in_collision = false;

    // for collision logging
    std::ofstream collision_file;
    double beginning_seconds;
    int collision_count = 0;


public:
    BehaviorController() : Node("behavior_controller") {
        // Declare and get topic names
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<std::string>("imu_topic", "/imu");
        this->declare_parameter<std::string>("joy_topic", "/joy");
        this->declare_parameter<std::string>("mux_topic", "/mux");
        this->declare_parameter<std::string>("keyboard_topic", "/key");
        this->declare_parameter<std::string>("brake_bool_topic", "/brake_bool");

        std::string scan_topic = this->get_parameter("scan_topic").as_string();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string joy_topic = this->get_parameter("joy_topic").as_string();
        std::string mux_topic = this->get_parameter("mux_topic").as_string();
        std::string keyboard_topic = this->get_parameter("keyboard_topic").as_string();
        std::string brake_bool_topic = this->get_parameter("brake_bool_topic").as_string();

        // Make a publisher for mux messages
        mux_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>(mux_topic, 10);

        // Start subscribers to listen to laser scan, joy, IMU, and odom messages
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 1, std::bind(&BehaviorController::laser_callback, this, _1));
        joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            joy_topic, 1, std::bind(&BehaviorController::joy_callback, this, _1));
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 1, std::bind(&BehaviorController::imu_callback, this, _1));
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 1, std::bind(&BehaviorController::odom_callback, this, _1));
        key_sub = this->create_subscription<std_msgs::msg::String>(
            keyboard_topic, 1, std::bind(&BehaviorController::key_callback, this, _1));
        brake_bool_sub = this->create_subscription<std_msgs::msg::Bool>(
            brake_bool_topic, 1, std::bind(&BehaviorController::brake_callback, this, _1));

        // Get mux indices
        this->declare_parameter<int>("joy_mux_idx", 0);
        this->declare_parameter<int>("key_mux_idx", 1);
        this->declare_parameter<int>("random_walker_mux_idx", 2);
        this->declare_parameter<int>("brake_mux_idx", 3);
        this->declare_parameter<int>("nav_mux_idx", 4);
        joy_mux_idx = this->get_parameter("joy_mux_idx").as_int();
        key_mux_idx = this->get_parameter("key_mux_idx").as_int();
        random_walker_mux_idx = this->get_parameter("random_walker_mux_idx").as_int();
        brake_mux_idx = this->get_parameter("brake_mux_idx").as_int();
        nav_mux_idx = this->get_parameter("nav_mux_idx").as_int();

        // Get button indices
        this->declare_parameter<int>("joy_button_idx", 4);
        this->declare_parameter<int>("key_button_idx", 6);
        this->declare_parameter<int>("random_walk_button_idx", 1);
        this->declare_parameter<int>("brake_button_idx", 0);
        this->declare_parameter<int>("nav_button_idx", 5);
        joy_button_idx = this->get_parameter("joy_button_idx").as_int();
        key_button_idx = this->get_parameter("key_button_idx").as_int();
        random_walk_button_idx = this->get_parameter("random_walk_button_idx").as_int();
        brake_button_idx = this->get_parameter("brake_button_idx").as_int();
        nav_button_idx = this->get_parameter("nav_button_idx").as_int();

        // Get key indices
        this->declare_parameter<std::string>("joy_key_char", "j");
        this->declare_parameter<std::string>("keyboard_key_char", "k");
        this->declare_parameter<std::string>("random_walk_key_char", "r");
        this->declare_parameter<std::string>("brake_key_char", "b");
        this->declare_parameter<std::string>("nav_key_char", "n");
        joy_key_char = this->get_parameter("joy_key_char").as_string();
        keyboard_key_char = this->get_parameter("keyboard_key_char").as_string();
        random_walk_key_char = this->get_parameter("random_walk_key_char").as_string();
        brake_key_char = this->get_parameter("brake_key_char").as_string();
        nav_key_char = this->get_parameter("nav_key_char").as_string();

        // Initialize the mux controller 
        this->declare_parameter<int>("mux_size", 5);
        mux_size = this->get_parameter("mux_size").as_int();
        mux_controller.resize(mux_size, false);

        // Start with ebrake off
        safety_on = false;

        // Initialize state
        state = {.x=0.0, .y=0.0, .theta=0.0, .velocity=0.0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};

        // Get params for precomputation and collision detection
        this->declare_parameter<double>("ttc_threshold", 0.01);
        this->declare_parameter<int>("scan_beams", 1080);
        this->declare_parameter<double>("scan_distance_to_base_link", 0.275);
        this->declare_parameter<double>("width", 0.2032);
        this->declare_parameter<double>("wheelbase", 0.3302);
        this->declare_parameter<double>("scan_field_of_view", 6.2831853);

        ttc_threshold = this->get_parameter("ttc_threshold").as_double();
        int scan_beams = this->get_parameter("scan_beams").as_int();
        double scan_distance_to_base_link = this->get_parameter("scan_distance_to_base_link").as_double();
        double width = this->get_parameter("width").as_double();
        double wheelbase = this->get_parameter("wheelbase").as_double();
        double scan_fov = this->get_parameter("scan_field_of_view").as_double();
        double scan_ang_incr = scan_fov / scan_beams;

        // Precompute cosine and distance to car at each angle of the laser scan
        cosines = Precompute::get_cosines(scan_beams, -scan_fov/2.0, scan_ang_incr);
        car_distances = Precompute::get_car_distances(scan_beams, wheelbase, width, 
                scan_distance_to_base_link, -scan_fov/2.0, scan_ang_incr);

        // Create collision file to be written to
        this->declare_parameter<std::string>("collision_file", "collision_file");
        std::string filename = this->get_parameter("collision_file").as_string();
        
        try {
            std::string package_share = ament_index_cpp::get_package_share_directory("f1tenth_simulator");
            collision_file.open(package_share + "/logs/" + filename + ".txt");
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Could not open collision log file: %s", e.what());
        }
        
        beginning_seconds = this->now().seconds();

        RCLCPP_INFO(this->get_logger(), "Behavior controller initialized");
    }

    /// ---------------------- GENERAL HELPER FUNCTIONS ----------------------

    void publish_mux() {
        // make mux message
        std_msgs::msg::Int32MultiArray mux_msg;
        mux_msg.data.clear();
        // push data onto message
        for (int i = 0; i < mux_size; i++) {
            mux_msg.data.push_back(int(mux_controller[i]));
        }

        // publish mux message
        mux_pub->publish(mux_msg);
    }

    void change_controller(int controller_idx) {
        // This changes the controller to the input index and publishes it

        // turn everything off
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }
        // turn on desired controller
        mux_controller[controller_idx] = true;

        publish_mux();
    }

    void collision_checker(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // This function calculates TTC to see if there's a collision
        if (state.velocity != 0) {
            for (size_t i = 0; i < msg->ranges.size(); i++) {
                double angle = msg->angle_min + i * msg->angle_increment;

                // calculate projected velocity
                double proj_velocity = state.velocity * cosines[i];
                double ttc = (msg->ranges[i] - car_distances[i]) / proj_velocity;

                // if it's small, there's a collision
                if ((ttc < ttc_threshold) && (ttc >= 0.0)) { 
                    // Send a blank mux and write to file
                    collision_helper();

                    in_collision = true;

                    collision_count++;
                    if (collision_file.is_open()) {
                        collision_file << "Collision #" << collision_count << " detected:\n";
                        collision_file << "TTC: " << ttc << " seconds\n";
                        collision_file << "Angle to obstacle: " << angle << " radians\n";
                        collision_file << "Time since start of sim: " << (this->now().seconds() - beginning_seconds) << " seconds\n";
                        collision_file << "\n";
                    }
                    return;
                }
            }
            // if it's gone through all beams without detecting a collision, reset in_collision
            in_collision = false;
        }
    }

    void collision_helper() {
        // This function will turn off ebrake, clear the mux and publish it

        safety_on = false;

        // turn everything off
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }

        publish_mux();
    }

    void toggle_mux(int mux_idx, std::string driver_name) {
        // This takes in an index and the name of the planner/driver and 
        // toggles the mux appropriately
        if (mux_controller[mux_idx]) {
            RCLCPP_INFO(this->get_logger(), "%s turned off", driver_name.c_str());
            mux_controller[mux_idx] = false;
            publish_mux();
        }
        else {
            RCLCPP_INFO(this->get_logger(), "%s turned on", driver_name.c_str());
            change_controller(mux_idx);
        }
    }

    void toggle_brake_mux() {
        RCLCPP_INFO(this->get_logger(), "Emergency brake engaged");
        // turn everything off
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }
        // turn on desired controller
        mux_controller[brake_mux_idx] = true;

        publish_mux();
    }


    /// ---------------------- CALLBACK FUNCTIONS ----------------------

    void brake_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && safety_on) {
            toggle_brake_mux();
        } else if (!msg->data && mux_controller[brake_mux_idx]) {
            mux_controller[brake_mux_idx] = false;
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // Changing mux_controller:
        if (msg->buttons[joy_button_idx]) { 
            // joystick
            toggle_mux(joy_mux_idx, "Joystick");
        }
        if (msg->buttons[key_button_idx]) { 
            // keyboard
            toggle_mux(key_mux_idx, "Keyboard");
        }
        else if (msg->buttons[brake_button_idx]) { 
            // emergency brake 
            if (safety_on) {
                RCLCPP_INFO(this->get_logger(), "Emergency brake turned off");
                safety_on = false;
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Emergency brake turned on");
                safety_on = true;
            }
        }
        else if (msg->buttons[random_walk_button_idx]) { 
            // random walker
            toggle_mux(random_walker_mux_idx, "Random Walker");
        } else if (msg->buttons[nav_button_idx]) {
            // nav
            toggle_mux(nav_mux_idx, "Navigation");
        }
    }

    void key_callback(const std_msgs::msg::String::SharedPtr msg) {
        // Changing mux controller:
        if (msg->data == joy_key_char) {
            // joystick
            toggle_mux(joy_mux_idx, "Joystick");
        } else if (msg->data == keyboard_key_char) {
            // keyboard
            toggle_mux(key_mux_idx, "Keyboard");
        } else if (msg->data == brake_key_char) {
            // emergency brake 
            if (safety_on) {
                RCLCPP_INFO(this->get_logger(), "Emergency brake turned off");
                safety_on = false;
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Emergency brake turned on");
                safety_on = true;
            }
        } else if (msg->data == random_walk_key_char) {
            // random walker
            toggle_mux(random_walker_mux_idx, "Random Walker");
        } else if (msg->data == nav_key_char) {
            // nav
            toggle_mux(nav_mux_idx, "Navigation");
        }
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // check for a collision
        collision_checker(msg);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Keep track of state to be used elsewhere
        state.velocity = msg->twist.twist.linear.x;
        state.angular_velocity = msg->twist.twist.angular.z;
        state.x = msg->pose.pose.position.x;
        state.y = msg->pose.pose.position.y;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        (void)msg;  // Unused parameter
    }

};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BehaviorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
