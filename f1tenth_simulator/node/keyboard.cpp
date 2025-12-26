#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <termios.h>

#include <cstdio>
#include <csignal>

// for printing
#include <iostream>

#include <memory>

static volatile sig_atomic_t keep_running = 1;


void sigHandler(int not_used) {
    (void)not_used;
    keep_running = 0;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    
    // Initialize Node
    auto node = std::make_shared<rclcpp::Node>("keyboard");

    // Declare and get parameter
    node->declare_parameter<std::string>("keyboard_topic", "/key");
    std::string keyboard_topic = node->get_parameter("keyboard_topic").as_string();

    // Initialize publisher
    auto key_pub = node->create_publisher<std_msgs::msg::String>(keyboard_topic, 10);


    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr(STDIN_FILENO, 0, &newt);

    struct sigaction act;
    act.sa_handler = sigHandler;
    sigemptyset(&act.sa_mask);
    act.sa_flags = 0;
    sigaction(SIGINT, &act, NULL);
    

    std_msgs::msg::String msg;
    int c;
    while ((rclcpp::ok()) && (keep_running)) {
        // get the character pressed
        c = getchar();

        // Publish the character 
        msg.data = static_cast<char>(c);
        key_pub->publish(msg);
    }

    tcsetattr(STDIN_FILENO, 0, &oldt);
    
    rclcpp::shutdown();
    return 0;
}