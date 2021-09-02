// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "Eigen/Core"
#include "nav2_smac_planner/smac_planner_hybrid.hpp"

#define BENCHMARK_TESTING

#ifdef BENCHMARK_TESTING
#include "sensor_msgs/msg/point_cloud2.hpp"
#endif

namespace nav2_smac_planner
{

#ifdef BENCHMARK_TESTING
rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_map_pub;
rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_node_pub;
#endif

using namespace std::chrono;  // NOLINT

SmacPlannerHybrid::SmacPlannerHybrid()
: _a_star(nullptr),
  _collision_checker(nullptr, 1),
  _smoother(nullptr),
  _costmap(nullptr),
  _costmap_downsampler(nullptr)
{
}

SmacPlannerHybrid::~SmacPlannerHybrid()
{
  RCLCPP_INFO(
    _logger, "Destroying plugin %s of type SmacPlannerHybrid",
    _name.c_str());
}

void SmacPlannerHybrid::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  _logger = node->get_logger();
  _clock = node->get_clock();
  _costmap = costmap_ros->getCostmap();
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();

  bool allow_unknown;
  int max_iterations;
  int angle_quantizations;
  double lookup_table_size;
  SearchInfo search_info;
  std::string motion_model_for_search;

  // General planner params
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsample_costmap", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".downsample_costmap", _downsample_costmap);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsampling_factor", rclcpp::ParameterValue(1));
  node->get_parameter(name + ".downsampling_factor", _downsampling_factor);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".angle_quantization_bins", rclcpp::ParameterValue(72));
  node->get_parameter(name + ".angle_quantization_bins", angle_quantizations);
  _angle_bin_size = 2.0 * M_PI / angle_quantizations;
  _angle_quantizations = static_cast<unsigned int>(angle_quantizations);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_iterations", rclcpp::ParameterValue(1000000));
  node->get_parameter(name + ".max_iterations", max_iterations);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minimum_turning_radius", rclcpp::ParameterValue(0.4));
  node->get_parameter(name + ".minimum_turning_radius", search_info.minimum_turning_radius);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cache_obstacle_heuristic", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".cache_obstacle_heuristic", search_info.cache_obstacle_heuristic);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".reverse_penalty", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".reverse_penalty", search_info.reverse_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".change_penalty", rclcpp::ParameterValue(0.15));
  node->get_parameter(name + ".change_penalty", search_info.change_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".change_reverse_penalty", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".change_reverse_penalty", search_info.change_reverse_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".non_straight_penalty", rclcpp::ParameterValue(1.50));
  node->get_parameter(name + ".non_straight_penalty", search_info.non_straight_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cost_penalty", rclcpp::ParameterValue(1.7));
  node->get_parameter(name + ".cost_penalty", search_info.cost_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_ratio", rclcpp::ParameterValue(3.5));
  node->get_parameter(name + ".analytic_expansion_ratio", search_info.analytic_expansion_ratio);
  // default is inf to preserve original behavior. If confirmed to be used, M_PI*0.32 is a good default
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_analytic_expansion_angle_range", rclcpp::ParameterValue(std::numeric_limits<double>::infinity()));
  node->get_parameter(name + ".max_analytic_expansion_angle_range", search_info.max_analytic_expansion_angle_range);
  // default is inf to preserve original behavior. If confirmed to be used, 0.15 is a good default
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_analytic_expansion_cost_subelevation", rclcpp::ParameterValue(std::numeric_limits<double>::infinity()));
  node->get_parameter(name + ".max_analytic_expansion_cost_subelevation", search_info.max_analytic_expansion_cost_subelevation);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_analytic_expansion_length", rclcpp::ParameterValue(std::numeric_limits<double>::infinity()));
  node->get_parameter(name + ".max_analytic_expansion_length", search_info.max_analytic_expansion_length);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".obstacle_heuristic_enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".obstacle_heuristic_enabled", search_info.obstacle_heuristic_enabled);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".obstacle_heuristic_admissible", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".obstacle_heuristic_admissible", search_info.obstacle_heuristic_admissible);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_planning_time", rclcpp::ParameterValue(5.0));
  node->get_parameter(name + ".max_planning_time", _max_planning_time);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lookup_table_size", rclcpp::ParameterValue(20.0));
  node->get_parameter(name + ".lookup_table_size", lookup_table_size);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".motion_model_for_search", rclcpp::ParameterValue(std::string("DUBIN")));
  node->get_parameter(name + ".motion_model_for_search", motion_model_for_search);
  MotionModel motion_model = fromString(motion_model_for_search);
  if (motion_model == MotionModel::UNKNOWN) {
    RCLCPP_WARN(
      _logger,
      "Unable to get MotionModel search type. Given '%s', "
      "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP, STATE_LATTICE.",
      motion_model_for_search.c_str());
  }

  if (max_iterations <= 0) {
    RCLCPP_INFO(
      _logger, "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    max_iterations = std::numeric_limits<int>::max();
  }

  // convert to grid coordinates
  const double minimum_turning_radius_global_coords = search_info.minimum_turning_radius;
  search_info.minimum_turning_radius =
    search_info.minimum_turning_radius / (_costmap->getResolution() * _downsampling_factor);
  float lookup_table_dim =
    static_cast<float>(lookup_table_size) /
    static_cast<float>(_costmap->getResolution() * _downsampling_factor);

  // Make sure its a whole number
  lookup_table_dim = static_cast<float>(static_cast<int>(lookup_table_dim));

  // Make sure its an odd number
  if (static_cast<int>(lookup_table_dim) % 2 == 0) {
    RCLCPP_INFO(
      _logger,
      "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
      lookup_table_dim);
    lookup_table_dim += 1.0;
  }

  // Initialize collision checker
  _collision_checker = GridCollisionChecker(_costmap, _angle_quantizations);
  _collision_checker.setFootprint(
    costmap_ros->getRobotFootprint(),
    costmap_ros->getUseRadius(),
    findCircumscribedCost(costmap_ros));

  // Initialize A* template
  _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(motion_model, search_info);
  _a_star->initialize(
    allow_unknown,
    max_iterations,
    std::numeric_limits<int>::max(),
    lookup_table_dim,
    _angle_quantizations);

  // Initialize path smoother
  SmootherParams params;
  params.get(node, name);
  _smoother = std::make_unique<Smoother>(params);
  _smoother->initialize(minimum_turning_radius_global_coords);

  // Initialize costmap downsampler
  if (_downsample_costmap && _downsampling_factor > 1) {
    std::string topic_name = "downsampled_costmap";
    _costmap_downsampler = std::make_unique<CostmapDownsampler>();
    _costmap_downsampler->on_configure(
      node, _global_frame, topic_name, _costmap, _downsampling_factor);
  }

  _raw_plan_publisher = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);

  #ifdef BENCHMARK_TESTING
  pcl_map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "astar_map_pcl",
      rclcpp::QoS(10));
  pcl_node_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "astar_node_pcl",
      rclcpp::QoS(10));
  #endif

  RCLCPP_INFO(
    _logger, "Configured plugin %s of type SmacPlannerHybrid with "
    "maximum iterations %i, and %s. Using motion model: %s.",
    _name.c_str(), max_iterations,
    allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    toString(motion_model).c_str());
}

void SmacPlannerHybrid::activate()
{
  RCLCPP_INFO(
    _logger, "Activating plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _raw_plan_publisher->on_activate();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_activate();
  }
  #ifdef BENCHMARK_TESTING
  pcl_map_pub->on_activate();
  pcl_node_pub->on_activate();
  #endif
}

void SmacPlannerHybrid::deactivate()
{
  RCLCPP_INFO(
    _logger, "Deactivating plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _raw_plan_publisher->on_deactivate();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_deactivate();
  }
  #ifdef BENCHMARK_TESTING
  pcl_map_pub->on_deactivate();
  pcl_node_pub->on_deactivate();
  #endif
}

void SmacPlannerHybrid::cleanup()
{
  RCLCPP_INFO(
    _logger, "Cleaning up plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _a_star.reset();
  _smoother.reset();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_cleanup();
    _costmap_downsampler.reset();
  }
  _raw_plan_publisher.reset();
}

#ifdef BENCHMARK_TESTING
void publishOHMap() {
  sensor_msgs::msg::PointCloud2 pcl_map;
  sensor_msgs::msg::PointCloud2 pcl_node;

  std::vector<sensor_msgs::msg::PointField> fields;
  auto pf_x = sensor_msgs::msg::PointField();
  pf_x.name = std::string("x");
  pf_x.offset = 0;
  pf_x.datatype = (uint8_t)7;
  pf_x.count = 1;
  fields.push_back(pf_x);

  auto pf_y = sensor_msgs::msg::PointField();
  pf_y.name = std::string("y");
  pf_y.offset = sizeof(float);
  pf_y.datatype = (uint8_t)7;
  pf_y.count = 1;
  fields.push_back(pf_y);

  auto pf_z = sensor_msgs::msg::PointField();
  pf_z.name = std::string("z");
  pf_z.offset = sizeof(float)*2;
  pf_z.datatype = (uint8_t)7;
  pf_z.count = 1;
  fields.push_back(pf_z);

  auto pf_i = sensor_msgs::msg::PointField();
  pf_i.name = std::string("intensity");
  pf_i.offset = sizeof(float)*3;
  pf_i.datatype = (uint8_t)7;
  pf_i.count = 1;
  fields.push_back(pf_i);

  pcl_map.fields = fields;
  pcl_map.header.frame_id = "map";
  pcl_map.point_step = sizeof(PointXYZI);
  pcl_map.height = 1;
  
  pcl_node = pcl_map;

  std::vector<PointXYZI> points;
  points.reserve(NodeHybrid::map_points.size());
  for (auto &p : NodeHybrid::map_points)
    if (p.intensity > 0)
      points.push_back(p);

  pcl_map.data.assign((u_char *)points.data(), (u_char *)(points.data() + points.size()));
  pcl_map.width = points.size();
  pcl_map.row_step = pcl_map.width*pcl_map.point_step;
  pcl_map_pub->publish(pcl_map);

  points.clear();
  points.reserve(NodeHybrid::node_points.size());
  for (auto &p : NodeHybrid::node_points)
    if (p.intensity > 0)
      points.push_back(p);
  pcl_node.data.assign((u_char *)points.data(), (u_char *)(points.data() + points.size()));
  pcl_node.width = points.size();
  pcl_node.row_step = pcl_node.width*pcl_node.point_step;
  pcl_node_pub->publish(pcl_node);
  // std::string path = node.motion_table.obstacle_heuristic_admissible ? "~/oh_map_admissible.map" : "~/oh_map.map";
  // std::ofstream fout;
  // fout.open(path, std::ios::binary | std::ios::out);

  // int size_x = NodeHybrid::sampled_costmap->getSizeInCellsX();
  // int size_y = NodeHybrid::sampled_costmap->getSizeInCellsY();

  // fout.write((char*)&size_x, sizeof(size_x)); //TODO: write must be used because << converts value to ASCII string
  // fout.write((char*)&size_y, sizeof(size_y));
  // fout.write((char*)NodeHybrid::obstacle_heuristic_lookup_table.data(), size_x*size_y*sizeof(float));
  // fout.close();
}
#endif

nav_msgs::msg::Path SmacPlannerHybrid::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  steady_clock::time_point a = steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Downsample costmap, if required
  nav2_costmap_2d::Costmap2D * costmap = _costmap;
  if (_costmap_downsampler) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
    _collision_checker.setCostmap(costmap);
  }

  // Set collision checker and costmap information
  _a_star->setCollisionChecker(&_collision_checker);

  // Set starting point, in A* bin search coordinates
  unsigned int mx, my;
  costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  double orientation_bin = tf2::getYaw(start.pose.orientation) / _angle_bin_size;
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  unsigned int orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
  _a_star->setStart(mx, my, orientation_bin_id);

  // Set goal point, in A* bin search coordinates
  costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
  orientation_bin = tf2::getYaw(goal.pose.orientation) / _angle_bin_size;
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
  _a_star->setGoal(mx, my, orientation_bin_id);

  // Setup message
  nav_msgs::msg::Path plan;
  plan.header.stamp = _clock->now();
  plan.header.frame_id = _global_frame;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // Compute plan
  NodeHybrid::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  try {
    if (!_a_star->createPath(path, num_iterations, 0.0)) {
      if (num_iterations < _a_star->getMaxIterations()) {
        error = std::string("no valid path found");
      } else {
        error = std::string("exceeded maximum iterations");
      }
    }
  } catch (const std::runtime_error & e) {
    error = "invalid use: ";
    error += e.what();
  }

  if (!error.empty()) {
    RCLCPP_WARN(
      _logger,
      "%s: failed to create plan, %s.",
      _name.c_str(), error.c_str());
    return plan;
  }

  // Convert to world coordinates
  plan.poses.reserve(path.size());
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = getWorldCoords(path[i].x, path[i].y, costmap);
    pose.pose.orientation = getWorldOrientation(path[i].theta, _angle_bin_size);
    plan.poses.push_back(pose);
  }

  // Publish raw path for debug
  if (_raw_plan_publisher->get_subscription_count() > 0) {
    _raw_plan_publisher->publish(plan);
  }

  // Find how much time we have left to do smoothing
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  double time_remaining = _max_planning_time - static_cast<double>(time_span.count());

#ifdef BENCHMARK_TESTING
  double length = 0;
  for (unsigned int i = 1; i < plan.poses.size(); i++)
    length += std::hypot(plan.poses[i].pose.position.x - plan.poses[i-1].pose.position.x, plan.poses[i].pose.position.y - plan.poses[i-1].pose.position.y);
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations to plan path of length " << length << " meters." << std::endl;
  publishOHMap();
#endif

  // Smooth plan
  if (num_iterations > 1 && plan.poses.size() > 6) {
    _smoother->smooth(plan, costmap, time_remaining);
  }

#ifdef BENCHMARK_TESTING
  steady_clock::time_point c = steady_clock::now();
  duration<double> time_span2 = duration_cast<duration<double>>(c - b);
  std::cout << "It took " << time_span2.count() * 1000 <<
    " milliseconds to smooth path." << std::endl;
#endif

  return plan;
}

}  // namespace nav2_smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_smac_planner::SmacPlannerHybrid, nav2_core::GlobalPlanner)
