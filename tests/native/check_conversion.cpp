#include <iostream>
#include <stdexcept>
#include <vector>
#include <intera_core_msgs/InteractionControlCommand.h>
#include <intera_core_msgs/JointCommand.h>
#include <intera_core_msgs/msg/interaction_control_command.hpp>
#include <intera_core_msgs/msg/joint_command.hpp>
#include <ros1_bridge/factory.hpp>

template<typename R1, typename R2>
void check(const R2 & source)
{
  using Factory = ros1_bridge::Factory<R1, R2>;
  R1 typed;
  Factory::convert_2_to_1(source, typed);
  const uint32_t length = ros::serialization::serializationLength(typed);
  if (Factory::length_2_as_1_stream(source) != length) {
    throw std::runtime_error("Serialized lengths differ");
  }
  std::vector<uint8_t> expected(length), actual(length);
  ros::serialization::OStream standard(expected.data(), length);
  ros::serialization::serialize(standard, typed);
  ros::serialization::OStream converted(actual.data(), length);
  Factory::convert_2_to_1(source, converted);
  if (expected != actual) {
    throw std::runtime_error("Converter bytes differ from native ROS 1 serialization");
  }
  ros::serialization::IStream input(actual.data(), length);
  R2 decoded;
  Factory::convert_1_to_2(input, decoded);
  if (source != decoded) {
    throw std::runtime_error("ROS 1 wire round trip changed ROS 2 fields");
  }
}

int main()
{
  intera_core_msgs::msg::JointCommand joints;
  check<intera_core_msgs::JointCommand>(joints);
  joints.header.stamp.sec = 123;
  joints.header.stamp.nanosec = 456789;
  joints.header.frame_id = "isolated";
  joints.mode = 1;
  joints.names = {"right_j0"};
  joints.position = {0.01};
  check<intera_core_msgs::JointCommand>(joints);
  intera_core_msgs::msg::InteractionControlCommand interaction;
  check<intera_core_msgs::InteractionControlCommand>(interaction);
  interaction.max_impedance = {true, false, true, false, true, false};
  interaction.k_impedance = {0., 1., 2., 3., 4., 5.};
  interaction.interaction_control_mode = {1, 2, 3, 1, 2, 3};
  interaction.header.stamp.sec = 123;
  interaction.header.frame_id = "isolated";
  check<intera_core_msgs::InteractionControlCommand>(interaction);
  std::cout << "PASS: native C++ interfaces, empty arrays, boolean arrays and exact ROS 1 wire serialization" << std::endl;
}
