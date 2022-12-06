/**
 * @file collision_avoidance.cpp
 * @author Joseph Pranadeer Reddy Katakam (jkatak73@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-12-05
 * 
 * @copyright 
 *  // Copyright 2016 Open Source Robotics Foundation, Inc.
    //
    // Licensed under the Apache License, Version 2.0 (the "License");
    // you may not use this file except in compliance with the License.
    // You may obtain a copy of the License at
    //
    //     http://www.apache.org/licenses/LICENSE-2.0
    //
    // Unless required by applicable law or agreed to in writing, software
    // distributed under the License is distributed on an "AS IS" BASIS,
    // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    // See the License for the specific language governing permissions and
    // limitations under the License.
  */

// libraries
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Using literals to the sensor messages types for more readability
using std::placeholders::_1;
// using namespace std::chrono_literals;

using LIDAR = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

/**
 * @brief Class for using turtlebot3 to move in gazebo world avoiding obstacles // from lidar topic
 * 
 */
class CollisionAvoidance : public rclcpp::Node {
 public:
 /**
  * @brief Construct a new Collision Avoidance object
  * 
  */
    CollisionAvoidance() :
        Node("collision_avoidance_system") {
            // Initialize the publisher and subscriber
            auto callback =
                    std::bind(&CollisionAvoidance::topic_callback, this, _1);
            lidar_data_subscriber = this->create_subscription<LIDAR>
                                        ("scan", 10, callback);
            velocity_data_publisher =
                    this->create_publisher<TWIST>("cmd_vel", 10);
        }

 private:
 /**
  * @brief Callback function for subscriber
  * 
  * @param msg 
  */
    void topic_callback(const LIDAR& msg) {
        // 200 readings, from right to left, from -57 to 57 degress
        // calculate new velocity cmd

        if (msg.header.stamp.sec == 0) {
            return;
        }
        // auto scan_data = msg.ranges;
        auto start_angle = 330;
        auto angle_range = 60;
        for (int i = start_angle; i < start_angle + angle_range; i++) {
            if (sccd tur    an_data[i % 360] < 0.8) {
                perform_action(0.0, 0.1);
            } else {
                perform_action(0.05, 0.0);
            }
        }
    }
    /**
     * @brief Publisher for sending 'velocity' data.
     * 
     * @param x_vel 
     * @param z_vel 
     */
    void perform_action(double x_vel, double z_vel) {
        auto vel = TWIST();
        vel.linear.x = x_vel;
        vel.angular.z = -z_vel;
        velocity_data_publisher->publish(vel);
    }

    // Private class members
    rclcpp::Subscription<LIDAR>::SharedPtr lidar_data_subscriber;
    rclcpp::Publisher<TWIST>::SharedPtr velocity_data_publisher;
};

/**
 * @brief Initializing ROS node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {
    // 1.) Initialize ROS 2 C++ client library
    rclcpp::init(argc, argv);

    // 2.) Start processing
    rclcpp::spin(std::make_shared<CollisionAvoidance>());

    // 3.) Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
