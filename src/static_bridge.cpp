// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <string>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

#include <yaml-cpp/yaml.h>

// include ROS 2
#include "rclcpp/rclcpp.hpp"

#include "ros1_bridge/bridge.hpp"

struct Topic{
  std::string name;
  std::string ros1_type;
  std::string ros2_type;
};

std::ostream& operator<<(std::ostream& os, const Topic& topic)
{
    std::cout << "name: " << topic.name << std::endl
              << "ROS1: " << topic.ros1_type << std::endl
              << "ROS2: " << topic.ros2_type << std::endl;
  return os;
}

std::vector<Topic> loadYAMLFile(std::string yaml_path){

  std::vector<Topic> topics;

  YAML::Node config = YAML::LoadFile(yaml_path);

  for (std::size_t i = 0; i < config["topics"].size(); i++)
  {
    const auto topic_dict = config["topics"][i];
    Topic topic;
    topic.name = topic_dict["name"].as<std::string>();
    topic.ros1_type = topic_dict["ros1_type"].as<std::string>();
    topic.ros2_type = topic_dict["ros2_type"].as<std::string>();
    topics.push_back(topic);
    std::cout << topic << std::endl;
  }

  return topics;
}

int main(int argc, char * argv[])
{
  // ROS 1 node
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_bridge");

  std::string yaml_file = ros2_node->declare_parameter("yaml_file").get<std::string>();
  const auto topics = loadYAMLFile(yaml_file);

  std::vector<ros1_bridge::BridgeHandles> handles;
  for (const auto & topic : topics)
  {
    // bridge one example topic
    const std::string & topic_name = topic.name;
    const std::string & ros1_type_name = topic.ros1_type;
    const std::string & ros2_type_name = topic.ros2_type;
    size_t queue_size = 10;

    auto handle = ros1_bridge::create_bidirectional_bridge(
      ros1_node, ros2_node, ros1_type_name, ros2_type_name, topic_name, queue_size);
    handles.push_back(handle);
  }

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node, std::chrono::milliseconds(1000));
  }

  return 0;
}
