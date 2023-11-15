/*
 * Title: ati_ft_sensor_wrench_pub.cpp
 *
 * Description: This file is part of ati_ft_sensor package. It publishes the
 * wrench data from the ATI force torque sensor. It also publishes the wrench
 * data to the faked_forces_controller based on a passed parameter, serving as a
 * generic proxy for admittance control using PickNik's admittance controller.
 * Follow the general template to replicate this for another FT sensor.
 *
 * Author: Emmanuel Akita
 * Email:  efakita@utexas.edu
 * Created: 4 Oct, 2023
 *
 */

#include <AtiFTSensor.h>
#include <chrono>

// ROS includes
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class ATIWrenchPublisher : public rclcpp::Node {
  // Constructor
 public:
  ATIWrenchPublisher() : Node("ati_wrench_publisher") {
    initializeNode();

    // Create a timer that calls the publish function every 10 milliseconds.
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ATIWrenchPublisher::publishSensorData, this));

    if (!sensor_.initialize(ip_address_)) {
      RCLCPP_ERROR(this->get_logger(), "Could not initialize the sensor.");
    }
  }

  // Destructor
  ~ATIWrenchPublisher() {
    // Stop the timer.
    timer_->cancel();
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  ati_ft_sensor::AtiFTSensor sensor_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_wrench_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      pub_fake_controller_wrench_;
  std::string frame_id_, ip_address_;
  bool fake_controller_wrench_;

  void initializeNode() {
    // Get the required parameters
    this->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ip_address", rclcpp::PARAMETER_STRING);
    this->declare_parameter("pub_faked_wrench", rclcpp::PARAMETER_BOOL);

    frame_id_ = this->get_parameter("frame_id").as_string();
    ip_address_ = this->get_parameter("ip_address").as_string();
    fake_controller_wrench_ = this->get_parameter("pub_faked_wrench").as_bool();

    // Create a publisher for the sensor data topic.
    pub_wrench_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
        "wrench_data", rclcpp::SensorDataQoS());

    if (fake_controller_wrench_)
      pub_fake_controller_wrench_ =
          this->create_publisher<std_msgs::msg::Float64MultiArray>(
              "faked_forces_controller/commands", 10);
  }

  void publishSensorData() {
    // Initialize data objects
    sensor_.setBias();
    double force[3];
    double torque[3];
    sensor_.getFT(&force[0], &torque[0]);

    // Publish the sensor data
    geometry_msgs::msg::WrenchStamped wrench_msg;
    wrench_msg.header.stamp = shared_from_this()->get_clock()->now();
    wrench_msg.header.frame_id = frame_id_;
    wrench_msg.wrench.force.x = force[0];
    wrench_msg.wrench.force.y = force[1];
    wrench_msg.wrench.force.z = force[2];
    wrench_msg.wrench.torque.x = torque[0];
    wrench_msg.wrench.torque.y = torque[1];
    wrench_msg.wrench.torque.z = torque[2];
    pub_wrench_->publish(wrench_msg);

    if (fake_controller_wrench_) {
      auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
      msg->data = {force[0],  force[1],  force[2],
                   torque[0], torque[1], torque[2]};
      pub_fake_controller_wrench_->publish(std::move(msg));
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ATIWrenchPublisher>());
  rclcpp::shutdown();
  return 0;
}
