#include <rclcpp/rclcpp.hpp>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

#include "f1tenth_simulator/channel.h"

#include <memory>
#include <functional>
#include <vector>
#include <string>
#include <iostream>

using std::placeholders::_1;

class Mux : public rclcpp::Node {
private:
    // Listen for mux messages
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mux_sub;

    // Listen for messages from joystick and keyboard
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub;

    // Publish drive data to simulator/car
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;

    // Mux indices
    int joy_mux_idx;
    int key_mux_idx;

    // Mux controller array
    std::vector<bool> mux_controller;
    int mux_size;

    // Channel array
    std::vector<std::shared_ptr<Channel>> channels;

    // Make Channel class have access to these private variables
    friend class Channel;

    // For printing
    std::vector<bool> prev_mux;

    // Params for joystick calculations
    int joy_speed_axis, joy_angle_axis;
    double max_speed, max_steering_angle;
    // For keyboard driving
    double prev_key_velocity = 0.0;
    double keyboard_speed;
    double keyboard_steer_ang;


public:
    Mux() : Node("mux_controller") {
        // Declare and get topic names
        this->declare_parameter<std::string>("drive_topic", "/drive");
        this->declare_parameter<std::string>("mux_topic", "/mux");
        this->declare_parameter<std::string>("joy_topic", "/joy");
        this->declare_parameter<std::string>("keyboard_topic", "/key");

        std::string drive_topic = this->get_parameter("drive_topic").as_string();
        std::string mux_topic = this->get_parameter("mux_topic").as_string();
        std::string joy_topic = this->get_parameter("joy_topic").as_string();
        std::string key_topic = this->get_parameter("keyboard_topic").as_string();

        // Make a publisher for drive messages
        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

        // Start a subscriber to listen to mux messages
        mux_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            mux_topic, 1, std::bind(&Mux::mux_callback, this, _1));

        // Start subscribers to listen to joy and keyboard messages
        joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            joy_topic, 1, std::bind(&Mux::joy_callback, this, _1));
        key_sub = this->create_subscription<std_msgs::msg::String>(
            key_topic, 1, std::bind(&Mux::key_callback, this, _1));

        // get mux indices
        this->declare_parameter<int>("joy_mux_idx", 0);
        this->declare_parameter<int>("key_mux_idx", 1);
        joy_mux_idx = this->get_parameter("joy_mux_idx").as_int();
        key_mux_idx = this->get_parameter("key_mux_idx").as_int();

        // get params for joystick calculations
        this->declare_parameter<int>("joy_speed_axis", 3);
        this->declare_parameter<int>("joy_angle_axis", 2);
        this->declare_parameter<double>("max_steering_angle", 0.4189);
        this->declare_parameter<double>("max_speed", 7.0);
        joy_speed_axis = this->get_parameter("joy_speed_axis").as_int();
        joy_angle_axis = this->get_parameter("joy_angle_axis").as_int();
        max_steering_angle = this->get_parameter("max_steering_angle").as_double();
        max_speed = this->get_parameter("max_speed").as_double();

        // get params for keyboard driving
        this->declare_parameter<double>("keyboard_speed", 1.8);
        this->declare_parameter<double>("keyboard_steer_ang", 0.3);
        keyboard_speed = this->get_parameter("keyboard_speed").as_double();
        keyboard_steer_ang = this->get_parameter("keyboard_steer_ang").as_double();

        // get size of mux
        this->declare_parameter<int>("mux_size", 5);
        mux_size = this->get_parameter("mux_size").as_int();

        // initialize mux controller
        mux_controller.resize(mux_size, false);
        prev_mux.resize(mux_size, false);

        // A channel contains a subscriber to the given drive topic and a publisher to the main drive topic
        channels = std::vector<std::shared_ptr<Channel>>();

        /// Add new channels here:
        // Random driver example
        this->declare_parameter<std::string>("rand_drive_topic", "/rand_drive");
        this->declare_parameter<int>("random_walker_mux_idx", 2);
        std::string rand_drive_topic = this->get_parameter("rand_drive_topic").as_string();
        int random_walker_mux_idx = this->get_parameter("random_walker_mux_idx").as_int();
        add_channel(rand_drive_topic, drive_topic, random_walker_mux_idx);

        // Channel for emergency braking
        this->declare_parameter<std::string>("brake_drive_topic", "/brake");
        this->declare_parameter<int>("brake_mux_idx", 3);
        std::string brake_drive_topic = this->get_parameter("brake_drive_topic").as_string();
        int brake_mux_idx = this->get_parameter("brake_mux_idx").as_int();
        add_channel(brake_drive_topic, drive_topic, brake_mux_idx);

        // General navigation channel
        this->declare_parameter<std::string>("nav_drive_topic", "/nav");
        this->declare_parameter<int>("nav_mux_idx", 4);
        std::string nav_drive_topic = this->get_parameter("nav_drive_topic").as_string();
        int nav_mux_idx = this->get_parameter("nav_mux_idx").as_int();
        add_channel(nav_drive_topic, drive_topic, nav_mux_idx);

        RCLCPP_INFO(this->get_logger(), "Mux controller initialized");
    }

    void add_channel(std::string channel_name, std::string drive_topic, int mux_idx_) {
        auto new_channel = std::make_shared<Channel>(channel_name, drive_topic, mux_idx_, this);
        channels.push_back(new_channel);    
    }

    void publish_to_drive(double desired_velocity, double desired_steer) {
        // This will take in a desired velocity and steering angle and make and publish an 
        // AckermannDriveStamped message to the /drive topic

        // Make and publish message
        ackermann_msgs::msg::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::msg::AckermannDrive drive_msg;
        drive_msg.speed = desired_velocity;
        drive_msg.steering_angle = desired_steer;

        drive_st_msg.header.stamp = this->now();
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub->publish(drive_st_msg);
    }

    void mux_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        // reset mux member variable every time it's published
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = bool(msg->data[i]);
        }

        // Prints the mux whenever it is changed
        bool changed = false;
        // checks if nothing is on
        bool anything_on = false;
        for (int i = 0; i < mux_size; i++) {
            changed = changed || (mux_controller[i] != prev_mux[i]);
            anything_on = anything_on || mux_controller[i];
        }
        if (changed) {
            std::cout << "MUX: " << std::endl;
            for (int i = 0; i < mux_size; i++) {
                std::cout << mux_controller[i] << std::endl;
                prev_mux[i] = mux_controller[i];
            }
            std::cout << std::endl;
        }
        if (!anything_on) {
            // if no mux channel is active, halt the car
            publish_to_drive(0.0, 0.0);
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // make drive message from joystick if turned on
        if (mux_controller[joy_mux_idx]) {
            // Calculate desired velocity and steering angle
            double desired_velocity = max_speed * msg->axes[joy_speed_axis];
            double desired_steer = max_steering_angle * msg->axes[joy_angle_axis];

            publish_to_drive(desired_velocity, desired_steer);
        }
    }

    void key_callback(const std_msgs::msg::String::SharedPtr msg) {
        // make drive message from keyboard if turned on 
        if (mux_controller[key_mux_idx]) {
            // Determine desired velocity and steering angle
            double desired_velocity = 0.0;
            double desired_steer = 0.0;
            
            bool publish = true;

            if (msg->data == "w") {
                // Forward
                desired_velocity = keyboard_speed;
            } else if (msg->data == "s") {
                // Backwards
                desired_velocity = -keyboard_speed;
            } else if (msg->data == "a") {
                // Steer left and keep speed
                desired_steer = keyboard_steer_ang;
                desired_velocity = prev_key_velocity;
            } else if (msg->data == "d") {
                // Steer right and keep speed
                desired_steer = -keyboard_steer_ang;
                desired_velocity = prev_key_velocity;
            } else if (msg->data == " ") {
                // publish zeros to slow down/straighten out car
            } else {
                // so that it doesn't constantly publish zeros when you press other keys
                publish = false;
            }

            if (publish) {
                publish_to_drive(desired_velocity, desired_steer);
                prev_key_velocity = desired_velocity;
            }
        }
    }

    // Accessor for mux_controller (needed by Channel)
    bool get_mux_state(int idx) const {
        return mux_controller[idx];
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr get_drive_publisher() {
        return drive_pub;
    }

};


/// Channel class method implementations

Channel::Channel() {
    mp_mux = nullptr;
    mux_idx = -1;
}

Channel::Channel(std::string channel_name, std::string drive_topic, int mux_idx_, Mux* mux) 
: mux_idx(mux_idx_), mp_mux(mux) {
    drive_pub = mux->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    channel_sub = mux->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        channel_name, 1, std::bind(&Channel::drive_callback, this, std::placeholders::_1));
}

void Channel::drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    if (mp_mux->get_mux_state(this->mux_idx)) {
        drive_pub->publish(*msg);
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Mux>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}