#include <rclcpp/rclcpp.hpp>

// interactive marker
#include <interactive_markers/interactive_marker_server.hpp>

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "f1tenth_simulator/pose_2d.hpp"
#include "f1tenth_simulator/ackermann_kinematics.hpp"
#include "f1tenth_simulator/scan_simulator_2d.hpp"

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/car_params.hpp"
#include "f1tenth_simulator/ks_kinematics.hpp"
#include "f1tenth_simulator/st_kinematics.hpp"
#include "f1tenth_simulator/precompute.hpp"

#include <iostream>
#include <cmath>
#include <chrono>
#include <memory>
#include <functional>

using namespace racecar_simulator;
using namespace std::chrono_literals;
using std::placeholders::_1;

class RacecarSimulator : public rclcpp::Node {
private:
    // The transformation frames used
    std::string map_frame, base_frame, scan_frame;

    // obstacle states (1D index) and parameters
    std::vector<int> added_obs;
    // listen for clicked point for adding obstacles
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr obs_sub;
    int obstacle_size;

    // interactive markers' server
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server;

    // The car state and parameters
    CarState state;
    double previous_seconds;
    double scan_distance_to_base_link;
    double max_speed, max_steering_angle;
    double max_accel, max_steering_vel, max_decel;
    double desired_speed, desired_steer_ang;
    double accel, steer_angle_vel;
    CarParams params;
    double width;

    // A simulator of the laser
    ScanSimulator2D scan_simulator;
    double map_free_threshold;

    // For publishing transformations
    std::shared_ptr<tf2_ros::TransformBroadcaster> br;

    // A timer to update the pose
    rclcpp::TimerBase::SharedPtr update_pose_timer;

    // Listen for drive commands
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub;

    // Listen for a map
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    bool map_exists = false;

    // Listen for updates to the pose
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_rviz_sub;

    // Publish a scan, odometry, and imu data
    bool broadcast_transform;
    bool pub_gt_pose;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

    // publisher for map with obstacles
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;

    // keep an original map for obstacles
    nav_msgs::msg::OccupancyGrid original_map;
    nav_msgs::msg::OccupancyGrid current_map;

    // for obstacle collision
    int map_width, map_height;
    double map_resolution, origin_x, origin_y;

    // safety margin for collisions
    double thresh;
    double speed_clip_diff;

    // precompute cosines of scan angles
    std::vector<double> cosines;

    // scan parameters
    double scan_fov;
    double scan_ang_incr;

    // pi
    const double PI = 3.1415;

    // precompute distance from lidar to edge of car for each beam
    std::vector<double> car_distances;

    // for collision check
    bool TTC = false;
    double ttc_threshold;

    // steering delay
    int buffer_length;
    std::vector<double> steering_buffer;

    // Flag for first map message
    bool map_received = false;

public:

    RacecarSimulator() : Node("racecar_simulator") {
        // Initialize car state and driving commands
        state = {.x=0, .y=0, .theta=0, .velocity=0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};
        accel = 0.0;
        steer_angle_vel = 0.0;
        desired_speed = 0.0;
        desired_steer_ang = 0.0;
        previous_seconds = this->now().seconds();

        // Declare and get parameters
        // Topic names
        this->declare_parameter<std::string>("drive_topic", "/drive");
        this->declare_parameter<std::string>("map_topic", "/map");
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<std::string>("pose_topic", "/pose");
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<std::string>("pose_rviz_topic", "/initialpose");
        this->declare_parameter<std::string>("imu_topic", "/imu");
        this->declare_parameter<std::string>("ground_truth_pose_topic", "/gt_pose");

        std::string drive_topic = this->get_parameter("drive_topic").as_string();
        std::string map_topic = this->get_parameter("map_topic").as_string();
        std::string scan_topic = this->get_parameter("scan_topic").as_string();
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string pose_rviz_topic = this->get_parameter("pose_rviz_topic").as_string();
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string gt_pose_topic = this->get_parameter("ground_truth_pose_topic").as_string();

        // Steering delay params
        this->declare_parameter<int>("buffer_length", 5);
        buffer_length = this->get_parameter("buffer_length").as_int();

        // Transformation frame names
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("scan_frame", "laser");
        map_frame = this->get_parameter("map_frame").as_string();
        base_frame = this->get_parameter("base_frame").as_string();
        scan_frame = this->get_parameter("scan_frame").as_string();

        // Car parameters
        this->declare_parameter<double>("wheelbase", 0.3302);
        this->declare_parameter<double>("update_pose_rate", 0.001);
        this->declare_parameter<int>("scan_beams", 1080);
        this->declare_parameter<double>("scan_field_of_view", 6.2831853);
        this->declare_parameter<double>("scan_std_dev", 0.01);
        this->declare_parameter<double>("map_free_threshold", 0.8);
        this->declare_parameter<double>("scan_distance_to_base_link", 0.275);
        this->declare_parameter<double>("max_speed", 7.0);
        this->declare_parameter<double>("max_steering_angle", 0.4189);
        this->declare_parameter<double>("max_accel", 7.51);
        this->declare_parameter<double>("max_decel", 8.26);
        this->declare_parameter<double>("max_steering_vel", 3.2);
        this->declare_parameter<double>("friction_coeff", 0.523);
        this->declare_parameter<double>("height_cg", 0.074);
        this->declare_parameter<double>("l_cg2rear", 0.17145);
        this->declare_parameter<double>("l_cg2front", 0.15875);
        this->declare_parameter<double>("C_S_front", 4.718);
        this->declare_parameter<double>("C_S_rear", 5.4562);
        this->declare_parameter<double>("moment_inertia", 0.04712);
        this->declare_parameter<double>("mass", 3.47);
        this->declare_parameter<double>("width", 0.2032);

        params.wheelbase = this->get_parameter("wheelbase").as_double();
        double update_pose_rate = this->get_parameter("update_pose_rate").as_double();
        int scan_beams = this->get_parameter("scan_beams").as_int();
        scan_fov = this->get_parameter("scan_field_of_view").as_double();
        double scan_std_dev = this->get_parameter("scan_std_dev").as_double();
        map_free_threshold = this->get_parameter("map_free_threshold").as_double();
        scan_distance_to_base_link = this->get_parameter("scan_distance_to_base_link").as_double();
        max_speed = this->get_parameter("max_speed").as_double();
        max_steering_angle = this->get_parameter("max_steering_angle").as_double();
        max_accel = this->get_parameter("max_accel").as_double();
        max_decel = this->get_parameter("max_decel").as_double();
        max_steering_vel = this->get_parameter("max_steering_vel").as_double();
        params.friction_coeff = this->get_parameter("friction_coeff").as_double();
        params.h_cg = this->get_parameter("height_cg").as_double();
        params.l_r = this->get_parameter("l_cg2rear").as_double();
        params.l_f = this->get_parameter("l_cg2front").as_double();
        params.cs_f = this->get_parameter("C_S_front").as_double();
        params.cs_r = this->get_parameter("C_S_rear").as_double();
        params.I_z = this->get_parameter("moment_inertia").as_double();
        params.mass = this->get_parameter("mass").as_double();
        width = this->get_parameter("width").as_double();

        // clip velocity
        this->declare_parameter<double>("speed_clip_diff", 0.0);
        speed_clip_diff = this->get_parameter("speed_clip_diff").as_double();

        // Determine if we should broadcast
        this->declare_parameter<bool>("broadcast_transform", true);
        this->declare_parameter<bool>("publish_ground_truth_pose", true);
        broadcast_transform = this->get_parameter("broadcast_transform").as_bool();
        pub_gt_pose = this->get_parameter("publish_ground_truth_pose").as_bool();

        // Get obstacle size parameter
        this->declare_parameter<int>("obstacle_size", 2);
        obstacle_size = this->get_parameter("obstacle_size").as_int();

        // Initialize a simulator of the laser scanner
        scan_simulator = ScanSimulator2D(
            scan_beams,
            scan_fov,
            scan_std_dev);

        // Make a publisher for laser scan messages
        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 1);

        // Make a publisher for odometry messages
        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1);

        // Make a publisher for IMU messages
        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 1);

        // Make a publisher for publishing map with obstacles
        map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);

        // Make a publisher for ground truth pose
        pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(gt_pose_topic, 1);

        // Initialize transform broadcaster
        br = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Start a timer to output the pose
        auto timer_period = std::chrono::duration<double>(update_pose_rate);
        update_pose_timer = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
            std::bind(&RacecarSimulator::update_pose, this));

        // Start a subscriber to listen to drive commands
        drive_sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 1, std::bind(&RacecarSimulator::drive_callback, this, _1));

        // Start a subscriber to listen to new maps
        map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic, 1, std::bind(&RacecarSimulator::map_callback, this, _1));

        // Start a subscriber to listen to pose messages
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, 1, std::bind(&RacecarSimulator::pose_callback, this, _1));
        pose_rviz_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_rviz_topic, 1, std::bind(&RacecarSimulator::pose_rviz_callback, this, _1));

        // obstacle subscriber
        obs_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 1, std::bind(&RacecarSimulator::obs_callback, this, _1));

        // get collision safety margin
        this->declare_parameter<double>("coll_threshold", 0.0);
        this->declare_parameter<double>("ttc_threshold", 0.01);
        thresh = this->get_parameter("coll_threshold").as_double();
        ttc_threshold = this->get_parameter("ttc_threshold").as_double();

        scan_ang_incr = scan_simulator.get_angle_increment();

        cosines = Precompute::get_cosines(scan_beams, -scan_fov/2.0, scan_ang_incr);
        car_distances = Precompute::get_car_distances(scan_beams, params.wheelbase, width, 
                scan_distance_to_base_link, -scan_fov/2.0, scan_ang_incr);

        // steering delay buffer
        steering_buffer = std::vector<double>(buffer_length);

        // Initialize interactive marker server
        im_server = std::make_shared<interactive_markers::InteractiveMarkerServer>(
            "racecar_sim",
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_topics_interface(),
            this->get_node_services_interface());

        RCLCPP_INFO(this->get_logger(), "Simulator constructed. Waiting for map...");
    }

    void setup_interactive_markers() {
        // create button for clearing obstacles
        visualization_msgs::msg::InteractiveMarker clear_obs_button;
        clear_obs_button.header.frame_id = "map";
        clear_obs_button.pose.position.x = 0;
        clear_obs_button.pose.position.y = -5;
        clear_obs_button.scale = 1;
        clear_obs_button.name = "clear_obstacles";
        clear_obs_button.description = "Clear Obstacles\n(Left Click)";
        
        visualization_msgs::msg::InteractiveMarkerControl clear_obs_control;
        clear_obs_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
        clear_obs_control.name = "clear_obstacles_control";
        
        // make a box for the button
        visualization_msgs::msg::Marker clear_obs_marker;
        clear_obs_marker.type = visualization_msgs::msg::Marker::CUBE;
        clear_obs_marker.scale.x = clear_obs_button.scale * 0.45;
        clear_obs_marker.scale.y = clear_obs_button.scale * 0.65;
        clear_obs_marker.scale.z = clear_obs_button.scale * 0.45;
        clear_obs_marker.color.r = 0.0;
        clear_obs_marker.color.g = 1.0;
        clear_obs_marker.color.b = 0.0;
        clear_obs_marker.color.a = 1.0;

        clear_obs_control.markers.push_back(clear_obs_marker);
        clear_obs_control.always_visible = true;
        clear_obs_button.controls.push_back(clear_obs_control);

        im_server->insert(clear_obs_button);
        im_server->setCallback(clear_obs_button.name, 
            std::bind(&RacecarSimulator::clear_obstacles, this, _1));

        im_server->applyChanges();
    }

    void update_pose() {

        // simulate P controller
        compute_accel(desired_speed);
        double actual_ang = 0.0;
        if (static_cast<int>(steering_buffer.size()) < buffer_length) {
            steering_buffer.push_back(desired_steer_ang);
            actual_ang = 0.0;
        } else {
            steering_buffer.insert(steering_buffer.begin(), desired_steer_ang);
            actual_ang = steering_buffer.back();
            steering_buffer.pop_back();
        }
        set_steer_angle_vel(compute_steer_vel(actual_ang));

        // Update the pose
        rclcpp::Time timestamp = this->now();
        double current_seconds = timestamp.seconds();
        state = STKinematics::update(
            state,
            accel,
            steer_angle_vel,
            params,
            current_seconds - previous_seconds);
        state.velocity = std::min(std::max(state.velocity, -max_speed), max_speed);
        state.steer_angle = std::min(std::max(state.steer_angle, -max_steering_angle), max_steering_angle);

        previous_seconds = current_seconds;

        /// Publish the pose as a transformation
        pub_pose_transform(timestamp);

        /// Publish the steering angle as a transformation so the wheels move
        pub_steer_ang_transform(timestamp);

        // Make an odom message as well and publish it
        pub_odom(timestamp);

        // TODO: make and publish IMU message
        pub_imu(timestamp);


        /// KEEP in sim
        // If we have a map, perform a scan
        if (map_exists) {
            // Get the pose of the lidar, given the pose of base link
            // (base link is the center of the rear axle)
            Pose2D scan_pose;
            scan_pose.x = state.x + scan_distance_to_base_link * std::cos(state.theta);
            scan_pose.y = state.y + scan_distance_to_base_link * std::sin(state.theta);
            scan_pose.theta = state.theta;

            // Compute the scan from the lidar
            std::vector<double> scan = scan_simulator.scan(scan_pose);

            // Convert to float
            std::vector<float> scan_(scan.size());
            for (size_t i = 0; i < scan.size(); i++)
                scan_[i] = scan[i];

            // TTC Calculations are done here so the car can be halted in the simulator:
            // to reset TTC
            bool no_collision = true;
            if (state.velocity != 0) {
                for (size_t i = 0; i < scan_.size(); i++) {
                    // TTC calculations

                    // calculate projected velocity
                    double proj_velocity = state.velocity * cosines[i];
                    double ttc = (scan_[i] - car_distances[i]) / proj_velocity;
                    // if it's small enough to count as a collision
                    if ((ttc < ttc_threshold) && (ttc >= 0.0)) { 
                        if (!TTC) {
                            first_ttc_actions();
                        }

                        no_collision = false;
                        TTC = true;

                        RCLCPP_INFO(this->get_logger(), "Collision detected");
                    }
                }
            }

            // reset TTC
            if (no_collision)
                TTC = false;

            // Publish the laser message
            sensor_msgs::msg::LaserScan scan_msg;
            scan_msg.header.stamp = timestamp;
            scan_msg.header.frame_id = scan_frame;
            scan_msg.angle_min = -scan_simulator.get_field_of_view()/2.;
            scan_msg.angle_max =  scan_simulator.get_field_of_view()/2.;
            scan_msg.angle_increment = scan_simulator.get_angle_increment();
            scan_msg.range_max = 100;
            scan_msg.ranges = scan_;
            scan_msg.intensities = scan_;

            scan_pub->publish(scan_msg);


            // Publish a transformation between base link and laser
            pub_laser_link_transform(timestamp);

        }

    } // end of update_pose


        /// ---------------------- GENERAL HELPER FUNCTIONS ----------------------

    std::vector<int> ind_2_rc(int ind) {
        std::vector<int> rc;
        int row = floor(ind/map_width);
        int col = ind%map_width - 1;
        rc.push_back(row);
        rc.push_back(col);
        return rc;
    }

    int rc_2_ind(int r, int c) {
        return r*map_width + c;

    }

    std::vector<int> coord_2_cell_rc(double x, double y) {
        std::vector<int> rc;
        rc.push_back(static_cast<int>((y-origin_y)/map_resolution));
        rc.push_back(static_cast<int>((x-origin_x)/map_resolution));
        return rc;
    }

    void first_ttc_actions() {
        // completely stop vehicle
        state.velocity = 0.0;
        state.angular_velocity = 0.0;
        state.slip_angle = 0.0;
        state.steer_angle = 0.0;
        steer_angle_vel = 0.0;
        accel = 0.0;
        desired_speed = 0.0;
        desired_steer_ang = 0.0;
    }

    void set_accel(double accel_) {
        accel = std::min(std::max(accel_, -max_accel), max_accel);
    }

    void set_steer_angle_vel(double steer_angle_vel_) {
        steer_angle_vel = std::min(std::max(steer_angle_vel_, -max_steering_vel), max_steering_vel);
    }

    void add_obs(int ind) {
        std::vector<int> rc = ind_2_rc(ind);
        for (int i=-obstacle_size; i<obstacle_size; i++) {
            for (int j=-obstacle_size; j<obstacle_size; j++) {
                int current_r = rc[0]+i;
                int current_c = rc[1]+j;
                int current_ind = rc_2_ind(current_r, current_c);
                current_map.data[current_ind] = 100;
            }
        }
        map_pub->publish(current_map);
    }

    void clear_obs(int ind) {
        std::vector<int> rc = ind_2_rc(ind);
        for (int i=-obstacle_size; i<obstacle_size; i++) {
            for (int j=-obstacle_size; j<obstacle_size; j++) {
                int current_r = rc[0]+i;
                int current_c = rc[1]+j;
                int current_ind = rc_2_ind(current_r, current_c);
                current_map.data[current_ind] = 0;

            }
        }
        map_pub->publish(current_map);
    }

    double compute_steer_vel(double desired_angle) {
        // get difference between current and desired
        double dif = (desired_angle - state.steer_angle);

        // calculate velocity
        double steer_vel;
        if (std::abs(dif) > .0001)  // if the difference is not trivial
            steer_vel = dif / std::abs(dif) * max_steering_vel;
        else {
            steer_vel = 0;
        }

        return steer_vel;
    }

    void compute_accel(double desired_velocity) {
        // get difference between current and desired
        double dif = (desired_velocity - state.velocity);

        if (state.velocity > 0) {
            if (dif > 0) {
                // accelerate
                double kp = 2.0 * max_accel / max_speed;
                set_accel(kp * dif);
            } else {
                // brake
                accel = -max_decel; 
            }    
        } else if (state.velocity < 0) {
            if (dif > 0) {
                // brake
                accel = max_decel;

            } else {
                // accelerate
                double kp = 2.0 * max_accel / max_speed;
                set_accel(kp * dif);
            }   
        } else {
            // zero speed, accel either way
            double kp = 2.0 * max_accel / max_speed;
            set_accel(kp * dif);
        }
    }

        /// ---------------------- CALLBACK FUNCTIONS ----------------------

    void obs_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        double x = msg->point.x;
        double y = msg->point.y;
        std::vector<int> rc = coord_2_cell_rc(x, y);
        int ind = rc_2_ind(rc[0], rc[1]);
        added_obs.push_back(ind);
        add_obs(ind);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        state.x = msg->pose.position.x;
        state.y = msg->pose.position.y;
        geometry_msgs::msg::Quaternion q = msg->pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        state.theta = tf2::impl::getYaw(quat);
    }

    void pose_rviz_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        auto temp_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
        temp_pose->header = msg->header;
        temp_pose->pose = msg->pose.pose;
        pose_callback(temp_pose);
    }

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        desired_speed = msg->drive.speed;
        desired_steer_ang = msg->drive.steering_angle;
    }

    // button callbacks
    void clear_obstacles(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback) {
        bool clear_obs_clicked = false;
        if (feedback->event_type == 3) {
            clear_obs_clicked = true;
        }
        if (clear_obs_clicked) {
            RCLCPP_INFO(this->get_logger(), "Clearing obstacles.");
            current_map = original_map;
            map_pub->publish(current_map);

            clear_obs_clicked = false;
        }
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // Fetch the map parameters
        size_t height = msg->info.height;
        size_t width = msg->info.width;
        double resolution = msg->info.resolution;
        // Convert the ROS origin to a pose
        Pose2D origin;
        origin.x = msg->info.origin.position.x;
        origin.y = msg->info.origin.position.y;
        geometry_msgs::msg::Quaternion q = msg->info.origin.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        origin.theta = tf2::impl::getYaw(quat);

        // Convert the map to probability values
        std::vector<double> map(msg->data.size());
        for (size_t i = 0; i < height * width; i++) {
            if (msg->data[i] > 100 or msg->data[i] < 0) {
                map[i] = 0.5; // Unknown
            } else {
                map[i] = msg->data[i]/100.;
            }
        }

        // Send the map to the scanner
        scan_simulator.set_map(
            map,
            height,
            width,
            resolution,
            origin,
            map_free_threshold);
        map_exists = true;

        // Store map info for obstacles
        if (!map_received) {
            original_map = *msg;
            current_map = *msg;
            map_width = msg->info.width;
            map_height = msg->info.height;
            origin_x = msg->info.origin.position.x;
            origin_y = msg->info.origin.position.y;
            map_resolution = msg->info.resolution;
            map_received = true;

            // Setup interactive markers after receiving first map
            setup_interactive_markers();
            RCLCPP_INFO(this->get_logger(), "Map received. Simulator ready.");
        }
    }

        /// ---------------------- PUBLISHING HELPER FUNCTIONS ----------------------

    void pub_pose_transform(rclcpp::Time timestamp) {
        // Convert the pose into a transformation
        geometry_msgs::msg::Transform t;
        t.translation.x = state.x;
        t.translation.y = state.y;
        tf2::Quaternion quat;
        quat.setEuler(0., 0., state.theta);
        t.rotation.x = quat.x();
        t.rotation.y = quat.y();
        t.rotation.z = quat.z();
        t.rotation.w = quat.w();

        // publish ground truth pose
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = "map";
        ps.header.stamp = timestamp;
        ps.pose.position.x = state.x;
        ps.pose.position.y = state.y;
        ps.pose.orientation.x = quat.x();
        ps.pose.orientation.y = quat.y();
        ps.pose.orientation.z = quat.z();
        ps.pose.orientation.w = quat.w();

        // Add a header to the transformation
        geometry_msgs::msg::TransformStamped ts;
        ts.transform = t;
        ts.header.stamp = timestamp;
        ts.header.frame_id = map_frame;
        ts.child_frame_id = base_frame;

        // Publish them
        if (broadcast_transform) {
            br->sendTransform(ts);
        }
        if (pub_gt_pose) {
            pose_pub->publish(ps);
        }
    }

    void pub_steer_ang_transform(rclcpp::Time timestamp) {
        // Set the steering angle to make the wheels move
        // Publish the steering angle
        tf2::Quaternion quat_wheel;
        quat_wheel.setEuler(0., 0., state.steer_angle);
        geometry_msgs::msg::TransformStamped ts_wheel;
        ts_wheel.transform.rotation.x = quat_wheel.x();
        ts_wheel.transform.rotation.y = quat_wheel.y();
        ts_wheel.transform.rotation.z = quat_wheel.z();
        ts_wheel.transform.rotation.w = quat_wheel.w();
        ts_wheel.header.stamp = timestamp;
        ts_wheel.header.frame_id = "front_left_hinge";
        ts_wheel.child_frame_id = "front_left_wheel";
        br->sendTransform(ts_wheel);
        ts_wheel.header.frame_id = "front_right_hinge";
        ts_wheel.child_frame_id = "front_right_wheel";
        br->sendTransform(ts_wheel);
    }

    void pub_laser_link_transform(rclcpp::Time timestamp) {
        // Publish a transformation between base link and laser
        geometry_msgs::msg::TransformStamped scan_ts;
        scan_ts.transform.translation.x = scan_distance_to_base_link;
        scan_ts.transform.rotation.w = 1;
        scan_ts.header.stamp = timestamp;
        scan_ts.header.frame_id = base_frame;
        scan_ts.child_frame_id = scan_frame;
        br->sendTransform(scan_ts);
    }

    void pub_odom(rclcpp::Time timestamp) {
        // Make an odom message and publish it
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = timestamp;
        odom.header.frame_id = map_frame;
        odom.child_frame_id = base_frame;
        odom.pose.pose.position.x = state.x;
        odom.pose.pose.position.y = state.y;
        tf2::Quaternion quat;
        quat.setEuler(0., 0., state.theta);
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();
        odom.twist.twist.linear.x = state.velocity;
        // Compute lateral velocity from velocity and slip angle: v_y = v_x * tan(beta)
        odom.twist.twist.linear.y = state.velocity * std::tan(state.slip_angle);
        odom.twist.twist.angular.z = state.angular_velocity;
        odom_pub->publish(odom);
    }

    void pub_imu(rclcpp::Time timestamp) {
        // Make an IMU message and publish it
        // TODO: make imu message
        sensor_msgs::msg::Imu imu;
        imu.header.stamp = timestamp;
        imu.header.frame_id = map_frame;


        imu_pub->publish(imu);
    }

};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RacecarSimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
