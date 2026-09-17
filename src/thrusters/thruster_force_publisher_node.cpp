#include <tinyxml2.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace sura_hardware_interface
{
namespace
{

std::string strip_slashes(std::string value)
{
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }

  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }

  return value;
}

std::string namespaced_absolute_name(
  const std::string & robot_namespace,
  const std::string & suffix)
{
  if (robot_namespace.empty()) {
    return "/" + suffix;
  }

  return "/" + robot_namespace + "/" + suffix;
}

std::vector<std::string> extract_thruster_joints_from_ros2_control(
  const std::string & robot_description)
{
  tinyxml2::XMLDocument doc;
  if (doc.Parse(robot_description.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to parse robot_description XML");
  }

  const tinyxml2::XMLElement * robot = doc.FirstChildElement("robot");
  if (robot == nullptr) {
    throw std::runtime_error("robot_description does not contain a <robot> element");
  }

  std::vector<std::string> joint_names;
  for (
    const tinyxml2::XMLElement * ros2_control = robot->FirstChildElement("ros2_control");
    ros2_control != nullptr;
    ros2_control = ros2_control->NextSiblingElement("ros2_control"))
  {
    for (
      const tinyxml2::XMLElement * joint = ros2_control->FirstChildElement("joint");
      joint != nullptr;
      joint = joint->NextSiblingElement("joint"))
    {
      const char * joint_name = joint->Attribute("name");
      if (joint_name == nullptr) {
        continue;
      }

      bool has_effort_command_interface = false;
      for (
        const tinyxml2::XMLElement * command_interface = joint->FirstChildElement(
          "command_interface");
        command_interface != nullptr;
        command_interface = command_interface->NextSiblingElement("command_interface"))
      {
        const char * interface_name = command_interface->Attribute("name");
        if (interface_name != nullptr && std::string(interface_name) == "effort") {
          has_effort_command_interface = true;
          break;
        }
      }

      if (has_effort_command_interface) {
        joint_names.emplace_back(joint_name);
      }
    }
  }

  if (joint_names.empty()) {
    throw std::runtime_error("No effort command joints found under <ros2_control>");
  }

  return joint_names;
}

}  // namespace

class ThrusterForcePublisherNode : public rclcpp::Node
{
public:
  ThrusterForcePublisherNode()
  : Node("thruster_force_publisher")
  {
    robot_namespace_ = strip_slashes(declare_parameter<std::string>("robot_namespace", "cirtesub"));
    input_topic_ = declare_parameter<std::string>(
      "input_topic", namespaced_absolute_name(robot_namespace_, "controller/body_force/output"));
    output_topic_ = declare_parameter<std::string>(
      "output_topic", namespaced_absolute_name(robot_namespace_, "controller/thruster_forces"));
    const auto robot_description = declare_parameter<std::string>("robot_description", "");
    const auto robot_description_topic = declare_parameter<std::string>(
      "robot_description_topic", namespaced_absolute_name(robot_namespace_, "robot_description"));

    force_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      input_topic_,
      rclcpp::SystemDefaultsQoS(),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
      {
        publish_thruster_forces(*msg);
      });
    force_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      output_topic_,
      rclcpp::SystemDefaultsQoS());

    if (!robot_description.empty()) {
      configure_from_robot_description(robot_description);
    } else {
      robot_description_sub_ = create_subscription<std_msgs::msg::String>(
        robot_description_topic,
        rclcpp::QoS(1).transient_local().reliable(),
        [this](std_msgs::msg::String::ConstSharedPtr msg)
        {
          configure_from_robot_description(msg->data);
          robot_description_sub_.reset();
        });

      RCLCPP_INFO(
        get_logger(),
        "Waiting for robot_description on parameter 'robot_description' or topic '%s'",
        robot_description_topic.c_str());
    }
  }

private:
  struct ThrusterInfo
  {
    std::string joint_name;
    std::string frame_id;
  };

  void configure_from_robot_description(const std::string & robot_description)
  {
    if (configured_) {
      return;
    }

    std::vector<std::string> thruster_joints;
    try {
      thruster_joints = extract_thruster_joints_from_ros2_control(robot_description);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to configure thruster force publishers: %s", e.what());
      return;
    }

    std::vector<ThrusterInfo> thrusters;
    thrusters.reserve(thruster_joints.size());

    for (const auto & joint_name : thruster_joints) {
      ThrusterInfo info;
      info.joint_name = joint_name;
      info.frame_id = joint_name;

      RCLCPP_INFO(
        get_logger(),
        "Publishing %s local X force on '%s' in frame '%s'",
        joint_name.c_str(),
        output_topic_.c_str(),
        info.frame_id.c_str());

      thrusters.emplace_back(std::move(info));
    }

    thrusters_ = std::move(thrusters);
    configured_ = true;
  }

  void publish_thruster_forces(const std_msgs::msg::Float64MultiArray & msg)
  {
    if (!configured_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Ignoring thruster forces until robot_description is available");
      return;
    }

    if (msg.data.size() != thrusters_.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Expected %zu thruster force values, received %zu",
        thrusters_.size(),
        msg.data.size());
    }

    const auto stamp = now();
    const std::size_t count = std::min(msg.data.size(), thrusters_.size());
    for (std::size_t i = 0; i < count; ++i) {
      const auto & thruster = thrusters_[i];

      geometry_msgs::msg::WrenchStamped wrench;
      wrench.header.stamp = stamp;
      wrench.header.frame_id = thruster.frame_id;
      wrench.wrench.force.x = msg.data[i];
      wrench.wrench.force.y = 0.0;
      wrench.wrench.force.z = 0.0;
      wrench.wrench.torque.x = 0.0;
      wrench.wrench.torque.y = 0.0;
      wrench.wrench.torque.z = 0.0;

      force_pub_->publish(wrench);
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string robot_namespace_;
  bool configured_{false};
  std::vector<ThrusterInfo> thrusters_;

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr force_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
};

}  // namespace sura_hardware_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sura_hardware_interface::ThrusterForcePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
