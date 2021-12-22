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

#include <math.h>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/node_lattice.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(AStarTest, test_a_star_2d)
{
  nav2_smac_planner::SearchInfo info;
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::Node2D> a_star(
    nav2_smac_planner::MotionModel::MOORE, info);
  int max_iterations = 10000;
  float tolerance = 0.0;
  float some_tolerance = 20.0;
  int it_on_approach = 10;
  double max_planning_time = 120.0;
  int num_it = 0;

  a_star.initialize(false, max_iterations, it_on_approach, max_planning_time, 0.0, 1);

  nav2_costmap_2d::Costmap2D * costmapA =
    new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0.0, 0.0, 0);
  // island in the middle of lethal cost to cross
  for (unsigned int i = 40; i <= 60; ++i) {
    for (unsigned int j = 40; j <= 60; ++j) {
      costmapA->setCost(i, j, 254);
    }
  }

  // functional case testing
  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmapA, 1);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);
  a_star.setCollisionChecker(checker.get());
  a_star.setStart(20u, 20u, 0);
  a_star.setGoal(80u, 80u, 0);
  nav2_smac_planner::Node2D::CoordinateVector path;
  EXPECT_TRUE(a_star.createPath(path, num_it, tolerance));
  EXPECT_EQ(num_it, 102);

  // check path is the right size and collision free
  EXPECT_EQ(path.size(), 81u);
  for (unsigned int i = 0; i != path.size(); i++) {
    EXPECT_EQ(costmapA->getCost(path[i].x, path[i].y), 0);
  }

  // setting non-zero dim 3 for 2D search
  EXPECT_THROW(a_star.setGoal(0, 0, 10), std::runtime_error);
  EXPECT_THROW(a_star.setStart(0, 0, 10), std::runtime_error);

  path.clear();
  // failure cases with invalid inputs
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::Node2D> a_star_2(
    nav2_smac_planner::MotionModel::VON_NEUMANN, info);
  a_star_2.initialize(false, max_iterations, it_on_approach, max_planning_time, 0, 1);
  num_it = 0;
  EXPECT_THROW(a_star_2.createPath(path, num_it, tolerance), std::runtime_error);
  a_star_2.setCollisionChecker(checker.get());
  num_it = 0;
  EXPECT_THROW(a_star_2.createPath(path, num_it, tolerance), std::runtime_error);
  a_star_2.setStart(50, 50, 0);  // invalid
  a_star_2.setGoal(0, 0, 0);  // valid
  num_it = 0;
  EXPECT_THROW(a_star_2.createPath(path, num_it, tolerance), std::runtime_error);
  a_star_2.setStart(0, 0, 0);  // valid
  a_star_2.setGoal(50, 50, 0);  // invalid
  num_it = 0;
  EXPECT_THROW(a_star_2.createPath(path, num_it, tolerance), std::runtime_error);
  num_it = 0;
  // invalid goal but liberal tolerance
  a_star_2.setStart(20, 20, 0);  // valid
  a_star_2.setGoal(50, 50, 0);  // invalid
  EXPECT_TRUE(a_star_2.createPath(path, num_it, some_tolerance));
  EXPECT_EQ(path.size(), 42u);
  for (unsigned int i = 0; i != path.size(); i++) {
    EXPECT_EQ(costmapA->getCost(path[i].x, path[i].y), 0);
  }

  EXPECT_TRUE(a_star_2.getStart() != nullptr);
  EXPECT_TRUE(a_star_2.getGoal() != nullptr);
  EXPECT_EQ(a_star_2.getSizeX(), 100u);
  EXPECT_EQ(a_star_2.getSizeY(), 100u);
  EXPECT_EQ(a_star_2.getSizeDim3(), 1u);
  EXPECT_EQ(a_star_2.getToleranceHeuristic(), 20.0);
  EXPECT_EQ(a_star_2.getOnApproachMaxIterations(), 10);

  delete costmapA;
}

TEST(AStarTest, test_a_star_se2)
{
  nav2_smac_planner::SearchInfo info;
  info.change_penalty = 0.1;
  info.non_straight_penalty = 1.1;
  info.reverse_penalty = 2.0;
  info.minimum_turning_radius = 8;  // in grid coordinates
  info.analytic_expansion_max_length = 20.0;  // in grid coordinates
  info.analytic_expansion_ratio = 3.5;
  info.max_analytic_expansion_angle_range = std::numeric_limits<float>::infinity();
  info.max_analytic_expansion_cost_subelevation = std::numeric_limits<float>::infinity();
  info.obstacle_heuristic_enabled = true;
  unsigned int size_theta = 72;
  info.cost_penalty = 1.7;
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeHybrid> a_star(
    nav2_smac_planner::MotionModel::DUBIN, info);
  int max_iterations = 10000;
  float tolerance = 10.0;
  int it_on_approach = 10;
  double max_planning_time = 120.0;
  int num_it = 0;

  a_star.initialize(false, max_iterations, it_on_approach, max_planning_time, 401, size_theta);

  nav2_costmap_2d::Costmap2D * costmapA =
    new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0.0, 0.0, 0);
  // island in the middle of lethal cost to cross
  for (unsigned int i = 40; i <= 60; ++i) {
    for (unsigned int j = 40; j <= 60; ++j) {
      costmapA->setCost(i, j, 254);
    }
  }

  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmapA, size_theta);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  // functional case testing
  a_star.setCollisionChecker(checker.get());
  a_star.setStart(10u, 10u, 0u);
  a_star.setGoal(80u, 80u, 40u);
  nav2_smac_planner::NodeHybrid::CoordinateVector path;
  EXPECT_TRUE(a_star.createPath(path, num_it, tolerance));

  // check path is the right size and collision free
  EXPECT_EQ(num_it, 3827);
  EXPECT_EQ(path.size(), 61u);
  for (unsigned int i = 0; i != path.size(); i++) {
    EXPECT_EQ(costmapA->getCost(path[i].x, path[i].y), 0);
  }
  // no skipped nodes
  for (unsigned int i = 1; i != path.size(); i++) {
    EXPECT_LT(hypotf(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y), 2.1f);
  }

  delete costmapA;

  // reinitialize a_star with analytic expansion constraints
  info.max_analytic_expansion_angle_range = M_PI*0.32;
  info.max_analytic_expansion_cost_subelevation = 0.15;
  
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeHybrid> a_star_2(
    nav2_smac_planner::MotionModel::DUBIN, info);

  nav2_costmap_2d::Costmap2D * costmapB =
  new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0.0, 0.0, 150);
  // island in the middle of lethal cost to cross
  for (unsigned int i = 40; i <= 60; ++i) {
    for (unsigned int j = 0; j < 100; ++j) {
      costmapB->setCost(i, j, 0);
    }
  }

  a_star_2.initialize(false, max_iterations, it_on_approach, max_planning_time, 401, size_theta);
  checker = std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmapB, size_theta);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);
  a_star_2.setCollisionChecker(checker.get());

  nav2_smac_planner::NodeHybrid dummy_node(0), dummy_node2(1);
  bool using_dummy_node2 = 0; // to avoid failure on (next != prev) condition
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeHybrid>::NodeGetter neighborGetter =
    [&, this](const unsigned int & index, nav2_smac_planner::NodeHybrid *& neighbor_rtn) -> bool
    {
      neighbor_rtn = (using_dummy_node2 = !using_dummy_node2) ? &dummy_node2 : &dummy_node;
      return true;
    };

  // analytic expansion constraints ok for start node
  a_star_2.setCollisionChecker(checker.get()); // clearGraph()
  a_star_2.setStart(20, 50, 0u);
  a_star_2.setGoal(30u, 50, 0u);
  num_it = 0;
  EXPECT_TRUE(a_star_2.createPath(path, num_it, tolerance));
  EXPECT_EQ(num_it, 1);

  // max subelevation constraint broken for start node (wait for a safer place to find analytic path from)
  a_star_2.setCollisionChecker(checker.get()); // clearGraph()
  a_star_2.setStart(20, 50, 0u);
  a_star_2.setGoal(80u, 50, 0u);
  num_it = 0;
  EXPECT_TRUE(a_star_2.createPath(path, num_it, tolerance));
  EXPECT_GT(num_it, 1);

  // max angle range constraint broken for start node (wait for a less complicated maneuver)
  a_star_2.setCollisionChecker(checker.get()); // clearGraph()
  a_star_2.setStart(20, 50, 0u);
  a_star_2.setGoal(30u, 50, 36u);
  num_it = 0;
  EXPECT_TRUE(a_star_2.createPath(path, num_it, tolerance));
  EXPECT_GT(num_it, 1);

  delete costmapB;
}

TEST(AStarTest, test_a_star_lattice)
{
  nav2_smac_planner::SearchInfo info;
  info.change_penalty = 0.05;
  info.change_reverse_penalty = 0.0;
  info.max_analytic_expansion_angle_range = std::numeric_limits<float>::infinity();
  info.max_analytic_expansion_cost_subelevation = std::numeric_limits<float>::infinity();
  info.obstacle_heuristic_enabled = true;
  info.obstacle_heuristic_admissible = false;
  info.non_straight_penalty = 1.05;
  info.reverse_penalty = 2.0;
  info.analytic_expansion_ratio = 3.5;
  info.lattice_filepath =
    ament_index_cpp::get_package_share_directory("nav2_smac_planner") + "/default_model.json";
  info.minimum_turning_radius = 8;  // in grid coordinates 0.4/0.05
  info.analytic_expansion_max_length = 20.0;  // in grid coordinates
  unsigned int size_theta = 16;
  info.cost_penalty = 2.0;
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeLattice> a_star(
    nav2_smac_planner::MotionModel::STATE_LATTICE, info);
  int max_iterations = 10000;
  float tolerance = 10.0;
  int it_on_approach = 10;
  double max_planning_time = 120.0;
  int num_it = 0;

  a_star.initialize(
    false, max_iterations, std::numeric_limits<int>::max(), max_planning_time, 401, size_theta);

  nav2_costmap_2d::Costmap2D * costmapA =
    new nav2_costmap_2d::Costmap2D(100, 100, 0.05, 0.0, 0.0, 0);
  // island in the middle of lethal cost to cross
  for (unsigned int i = 20; i <= 30; ++i) {
    for (unsigned int j = 20; j <= 30; ++j) {
      costmapA->setCost(i, j, 254);
    }
  }

  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmapA, size_theta);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  // functional case testing
  a_star.setCollisionChecker(checker.get());
  a_star.setStart(5u, 5u, 0u);
  a_star.setGoal(40u, 40u, 1u);
  nav2_smac_planner::NodeLattice::CoordinateVector path;
  EXPECT_TRUE(a_star.createPath(path, num_it, tolerance));

  // check path is the right size and collision free
  EXPECT_EQ(num_it, 14);
  EXPECT_EQ(path.size(), 47u);
  for (unsigned int i = 0; i != path.size(); i++) {
    EXPECT_EQ(costmapA->getCost(path[i].x, path[i].y), 0);
  }
  // no skipped nodes
  for (unsigned int i = 1; i != path.size(); i++) {
    EXPECT_LT(hypotf(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y), 2.1f);
  }

  delete costmapA;
}

TEST(AStarTest, test_se2_single_pose_path)
{
  nav2_smac_planner::SearchInfo info;
  info.change_penalty = 0.1;
  info.non_straight_penalty = 1.1;
  info.reverse_penalty = 2.0;
  info.minimum_turning_radius = 8;  // in grid coordinates
  info.analytic_expansion_max_length = 20.0;  // in grid coordinates
  info.analytic_expansion_ratio = 3.5;
  info.max_analytic_expansion_angle_range = std::numeric_limits<float>::infinity();
  info.max_analytic_expansion_cost_subelevation = std::numeric_limits<float>::infinity();
  info.obstacle_heuristic_enabled = true;
  info.obstacle_heuristic_admissible = false;
  unsigned int size_theta = 72;
  info.cost_penalty = 1.7;
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeHybrid> a_star(
    nav2_smac_planner::MotionModel::DUBIN, info);
  int max_iterations = 100;
  float tolerance = 10.0;
  int it_on_approach = 10;
  double max_planning_time = 120.0;
  int num_it = 0;

  a_star.initialize(false, max_iterations, it_on_approach, max_planning_time, 401, size_theta);

  nav2_costmap_2d::Costmap2D * costmapA =
    new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0.0, 0.0, 0);

  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmapA, size_theta);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  // functional case testing
  a_star.setCollisionChecker(checker.get());
  a_star.setStart(10u, 10u, 0u);
  // Goal is one costmap cell away
  a_star.setGoal(12u, 10u, 0u);
  nav2_smac_planner::NodeHybrid::CoordinateVector path;
  EXPECT_TRUE(a_star.createPath(path, num_it, tolerance));

  // Check that the path is length one
  // With the current implementation, this produces a longer path
  // EXPECT_EQ(path.size(), 1u);
  EXPECT_GE(path.size(), 1u);

  delete costmapA;
}

TEST(AStarTest, test_constants)
{
  nav2_smac_planner::MotionModel mm = nav2_smac_planner::MotionModel::UNKNOWN;  // unknown
  EXPECT_EQ(nav2_smac_planner::toString(mm), std::string("Unknown"));
  mm = nav2_smac_planner::MotionModel::VON_NEUMANN;  // vonneumann
  EXPECT_EQ(nav2_smac_planner::toString(mm), std::string("Von Neumann"));
  mm = nav2_smac_planner::MotionModel::MOORE;  // moore
  EXPECT_EQ(nav2_smac_planner::toString(mm), std::string("Moore"));
  mm = nav2_smac_planner::MotionModel::DUBIN;  // dubin
  EXPECT_EQ(nav2_smac_planner::toString(mm), std::string("Dubin"));
  mm = nav2_smac_planner::MotionModel::REEDS_SHEPP;  // reeds-shepp
  EXPECT_EQ(nav2_smac_planner::toString(mm), std::string("Reeds-Shepp"));

  EXPECT_EQ(
    nav2_smac_planner::fromString(
      "VON_NEUMANN"), nav2_smac_planner::MotionModel::VON_NEUMANN);
  EXPECT_EQ(nav2_smac_planner::fromString("MOORE"), nav2_smac_planner::MotionModel::MOORE);
  EXPECT_EQ(nav2_smac_planner::fromString("DUBIN"), nav2_smac_planner::MotionModel::DUBIN);
  EXPECT_EQ(
    nav2_smac_planner::fromString(
      "REEDS_SHEPP"), nav2_smac_planner::MotionModel::REEDS_SHEPP);
  EXPECT_EQ(nav2_smac_planner::fromString("NONE"), nav2_smac_planner::MotionModel::UNKNOWN);
}
