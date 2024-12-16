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

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/compute_path_to_pose_service.hpp"

namespace nav2_behavior_tree
{

ComputePathToPoseService::ComputePathToPoseService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_rtv_msgs::srv::ComputePathToPose>(service_node_name, conf)
{
}

void ComputePathToPoseService::on_tick()
{
  getInput("goal", request_->goal);
  getInput("planner_id", request_->planner_id);
  if (getInput("start", request_->start)) {
    request_->use_start = true;
  }
}

BT::NodeStatus ComputePathToPoseService::on_completion(
  std::shared_ptr<nav2_rtv_msgs::srv::ComputePathToPose::Response> response)
{
  setOutput("path", response->path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ComputePathToPoseService>("ComputePathToPoseService");
}