#include <rclcpp/rclcpp.hpp>

// Publish to a topic with this message type
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/msg/ackermann_drive.hpp>

// Subscribe to a topic with this message type
#include <nav_msgs/msg/odometry.hpp>

// for printing
#include <iostream>

// for RAND_MAX
#include <cstdlib>

#include <memory>
#include <functional>

using std::placeholders::_1;

class RandomWalker : public rclcpp::Node {
private:
    // car parameters
    double max_speed, max_steering_angle;

    // Listen for odom messages
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    // Publish drive data
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;

    // previous desired steering angle
    double prev_angle = 0.0;


public:
    RandomWalker() : Node("random_walker") {
        // Declare and get topic names
        this->declare_parameter<std::string>("rand_drive_topic", "/rand_drive");
        this->declare_parameter<std::string>("odom_topic", "/odom");
        std::string drive_topic = this->get_parameter("rand_drive_topic").as_string();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();

        // Declare and get car parameters
        this->declare_parameter<double>("max_speed", 7.0);
        this->declare_parameter<double>("max_steering_angle", 0.4189);
        max_speed = this->get_parameter("max_speed").as_double();
        max_steering_angle = this->get_parameter("max_steering_angle").as_double();

        // Make a publisher for drive messages
        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

        // Start a subscriber to listen to odom messages
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 1, std::bind(&RandomWalker::odom_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Random walker initialized");
    }


    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        (void)msg;  // Unused parameter - publishing is done in odom callback just so it's at the same rate as the sim

        // initialize message to be published
        ackermann_msgs::msg::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::msg::AckermannDrive drive_msg;

        /// SPEED CALCULATION:
        // set constant speed to be half of max speed
        drive_msg.speed = max_speed / 2.0;


        /// STEERING ANGLE CALCULATION
        // random number between 0 and 1
        double random = ((double) rand() / RAND_MAX);
        // good range to cause lots of turning
        double range = max_steering_angle / 2.0;
        // compute random amount to change desired angle by (between -range and range)
        double rand_ang = range * random - range / 2.0;

        // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
        random = ((double) rand() / RAND_MAX);
        if ((random > .8) && (prev_angle != 0)) {
            double sign_rand = rand_ang / std::abs(rand_ang);
            double sign_prev = prev_angle / std::abs(prev_angle);
            rand_ang *= sign_rand * sign_prev;
        }

        // set angle (add random change to previous angle)
        drive_msg.steering_angle = std::min(std::max(prev_angle + rand_ang, -max_steering_angle), max_steering_angle);

        // reset previous desired angle
        prev_angle = drive_msg.steering_angle;

        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub->publish(drive_st_msg);
    }

}; // end of class definition


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RandomWalker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}