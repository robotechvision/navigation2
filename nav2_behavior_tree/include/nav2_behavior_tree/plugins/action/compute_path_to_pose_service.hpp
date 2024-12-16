// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__SERVICE__COMPUTE_PATH_TO_POSE_SERVICE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__SERVICE__COMPUTE_PATH_TO_POSE_SERVICE_HPP_

#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_rtv_msgs/srv/compute_path_to_pose.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that wraps nav2_msgs::srv::ComputePathToPoseService
 */
class ComputePathToPoseService : public BtServiceNode<nav2_rtv_msgs::srv::ComputePathToPose>
{
  using Service = nav2_rtv_msgs::srv::ComputePathToPose;

public:
  /**
   * @brief A constructor for nav2_behavior_tree::ComputePathToPoseService
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  ComputePathToPoseService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */
  void on_tick() override;

  BT::NodeStatus on_completion(std::shared_ptr<nav2_rtv_msgs::srv::ComputePathToPose::Response> response) override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("start", "Start pose of the path if overriding current robot pose"),
        BT::InputPort<std::string>("planner_id", "","Mapped name to the planner plugin type to use"),
        BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputePathToPose node"),
        BT::OutputPort<Service::Response::_error_code_type>("error_code_id", "The compute path to pose error code"),
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__SERVICE__COMPUTE_PATH_TO_POSE_SERVICE_HPP_
