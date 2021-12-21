// Copyright (c) 2021 RoboTech Vision
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

#ifndef NAV2_CERES_COSTAWARE_SMOOTHER__SMOOTHER_HPP_
#define NAV2_CERES_COSTAWARE_SMOOTHER__SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include "nav2_ceres_costaware_smoother/smoother_cost_function.hpp"
#include "nav2_ceres_costaware_smoother/utils.hpp"

#include "ceres/ceres.h"
#include "Eigen/Core"

namespace nav2_ceres_costaware_smoother
{

/**
 * @class nav2_smac_planner::Smoother
 * @brief A Conjugate Gradient 2D path smoother implementation
 */
class Smoother
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::Smoother
   */
  Smoother() {}

  /**
   * @brief A destructor for nav2_smac_planner::Smoother
   */
  ~Smoother() {}

  /**
   * @brief Initialization of the smoother
   * @param params OptimizerParam struct
   */
  void initialize(const OptimizerParams params)
  {
    _debug = params.debug;

    _options.linear_solver_type = ceres::DENSE_QR;

    _options.max_num_iterations = params.max_iterations;
    _options.max_solver_time_in_seconds = params.max_time;

    _options.function_tolerance = params.fn_tol;
    _options.gradient_tolerance = params.gradient_tol;
    _options.parameter_tolerance = params.param_tol;

    if (_debug) {
      _options.minimizer_progress_to_stdout = true;
      _options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
    } else {
      _options.logging_type = ceres::SILENT;
    }
  }

  /**
   * @brief Smoother method
   * @param path Reference to path
   * @param start_dir Orientation of the first pose
   * @param end_dir Orientation of the last pose
   * @param costmap Pointer to minimal costmap
   * @param params parameters weights
   * @return If smoothing was successful
   */
  bool smooth(
    std::vector<Eigen::Vector3d> & path,
    const Eigen::Vector2d & start_dir,
    const Eigen::Vector2d & end_dir,
    const nav2_costmap_2d::Costmap2D * costmap,
    const SmootherParams & params)
  {
    // Path has always at least 2 points
    CHECK(path.size() >= 2) << "Path must have at least 2 points";

    _options.max_solver_time_in_seconds = params.max_time;

    ceres::Problem problem;
    std::vector<Eigen::Vector3d> path_optim;
    std::vector<bool> optimized;
    if (buildProblem(path, costmap, params, problem, path_optim, optimized)) {
      // solve the problem
      ceres::Solver::Summary summary;
      ceres::Solve(_options, &problem, &summary);
      if (_debug)
        RCLCPP_INFO(rclcpp::get_logger("smoother_server"), "%s", summary.FullReport().c_str());
      if (!summary.IsSolutionUsable() || summary.initial_cost - summary.final_cost < 0.0) {
        return false;
      }
    }
    else
      RCLCPP_INFO(rclcpp::get_logger("smoother_server"), "Path too short to optimize");

    upsampleAndPopulate(path_optim, optimized, start_dir, end_dir, params, path);

    return true;
  }

private:

  /**
   * @brief Build problem method
   * @param path Reference to path
   * @param costmap Pointer to costmap
   * @param params Smoother parameters
   * @param problem Output problem to solve
   * @param path_optim Output path on which the problem will be solved
   * @param optimized False for points skipped by downsampling
   * @return If there is a problem to solve
   */
  bool buildProblem(
    const std::vector<Eigen::Vector3d> & path,
    const nav2_costmap_2d::Costmap2D * costmap,
    const SmootherParams & params,
    ceres::Problem &problem,
    std::vector<Eigen::Vector3d> & path_optim,
    std::vector<bool> &optimized)
  {
    // Create costmap grid
    _costmap_grid = std::make_shared<ceres::Grid2D<u_char>>(costmap->getCharMap(), 0, costmap->getSizeInCellsY(), 0, costmap->getSizeInCellsX());
    auto costmap_interpolator = std::make_shared<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>(*_costmap_grid);

    // Create residual blocks
    const double cusp_half_length = params.cusp_zone_length/2;
    ceres::LossFunction* loss_function = NULL;
    path_optim = path;
    optimized = std::vector<bool>(path.size());
    optimized[0] = true;
    int prelast_i = -1;
    int last_i = 0;
    double last_direction = path_optim[0][2];
    bool last_was_cusp = false;
    bool last_is_reversing = false;
    std::deque<std::pair<double, SmootherCostFunction *>> potential_cusp_funcs;
    double last_segment_len = EPSILON;
    double potential_cusp_funcs_len = 0;
    double len_since_cusp = std::numeric_limits<double>::infinity();

    for (int i = 1; i < (int)path_optim.size(); i++) {
      auto &pt = path_optim[i];
      bool is_cusp = false;
      if (i != (int)path_optim.size()-1) {
        is_cusp = pt[2]*last_direction < 0;
        last_direction = pt[2];

        // skip to downsample if can be skipped (no forward/reverse direction change)
        if (!is_cusp && i > 1 && i < (int)path_optim.size()-2 && (i - last_i) < params.input_downsampling_factor)
          continue;
      }

      // keep distance inequalities between poses (some might have been downsampled while others might not)
      double current_segment_len = (path_optim[i] - path_optim[last_i]).block<2, 1>(0, 0).norm();

      // forget cost functions which don't have chance to be part of a cusp zone
      potential_cusp_funcs_len += current_segment_len;
      while (!potential_cusp_funcs.empty() && potential_cusp_funcs_len > cusp_half_length) {
        potential_cusp_funcs_len -= potential_cusp_funcs.front().first;
        potential_cusp_funcs.pop_front();
      }

      // update cusp zone costmap weights
      if (is_cusp) {
        double len_to_cusp = current_segment_len;
        for (int i = potential_cusp_funcs.size()-1; i >= 0; i--) {
          auto &f = potential_cusp_funcs[i];
          double new_weight =
            params.cusp_costmap_weight*(1.0 - len_to_cusp/cusp_half_length) + params.costmap_weight*len_to_cusp/cusp_half_length;
          if (std::abs(new_weight - params.cusp_costmap_weight) < std::abs(f.second->costmapWeight() - params.cusp_costmap_weight))
            f.second->setCostmapWeight(new_weight);
          len_to_cusp += f.first;
        }
        potential_cusp_funcs_len = 0;
        potential_cusp_funcs.clear();
        len_since_cusp = 0;
      }

      // add cost function
      optimized[i] = true;
      if (prelast_i != -1) {
        bool is_in_cusp_zone = len_since_cusp <= cusp_half_length;
        SmootherCostFunction *cost_function = new SmootherCostFunction(
              path[last_i].template block<2, 1>(0, 0),
              (last_was_cusp ? -1 : 1)*last_segment_len/current_segment_len,
              last_is_reversing,
              costmap,
              costmap_interpolator,
              params,
              is_in_cusp_zone
                ? params.cusp_costmap_weight*(1.0 - len_since_cusp/cusp_half_length) + params.costmap_weight*len_since_cusp/cusp_half_length
                : params.costmap_weight);
        problem.AddResidualBlock(cost_function->AutoDiff(), loss_function, path_optim[last_i].data(), pt.data(), path_optim[prelast_i].data());
        
        // if (!is_in_cusp_zone)
        potential_cusp_funcs.emplace_back(current_segment_len, cost_function);
      }

      // shift current to last and last to pre-last
      last_was_cusp = is_cusp;
      last_is_reversing = last_direction < 0;
      prelast_i = last_i;
      last_i = i;
      len_since_cusp += current_segment_len;
      last_segment_len = std::max(EPSILON, current_segment_len);
    }

    if (problem.NumParameterBlocks() <= 4)
      return false; // nothing to optimize
    // first two and last two points are constant (to keep start and end direction)
    problem.SetParameterBlockConstant(path_optim.front().data());
    problem.SetParameterBlockConstant(path_optim[1].data());
    problem.SetParameterBlockConstant(path_optim[path_optim.size()-2].data());
    problem.SetParameterBlockConstant(path_optim.back().data());
    return true;
  }

  /**
   * @brief Populate optimized points to path, assigning orientations and upsampling poses using cubic bezier
   * @param path_optim Path with optimized points
   * @param optimized False for points skipped by downsampling
   * @param start_dir Orientation of the first pose
   * @param end_dir Orientation of the last pose
   * @param params Smoother parameters
   * @param path Output path with upsampled optimized points
   */
  void upsampleAndPopulate(
    const std::vector<Eigen::Vector3d> & path_optim,
    const std::vector<bool> &optimized,
    const Eigen::Vector2d & start_dir,
    const Eigen::Vector2d & end_dir,
    const SmootherParams & params,
    std::vector<Eigen::Vector3d> & path)
  {
    // Populate path, assign orientations, interpolate skipped/upsampled poses
    path.clear();
    path.emplace_back(path_optim[0][0], path_optim[0][1], atan2(start_dir[1], start_dir[0]));
    if (params.output_upsampling_factor > 1)
      path.reserve(params.output_upsampling_factor*(path_optim.size() - 1) + 1);
    int last_i = 0;
    int prelast_i = -1;
    Eigen::Vector2d prelast_dir = start_dir;
    for (int i = 1; i <= (int)path_optim.size(); i++) {
      if (i == (int)path_optim.size() || optimized[i]) {
        if (prelast_i != -1) {
          Eigen::Vector2d last_dir;
          auto &prelast = path_optim[prelast_i];
          auto &last = path_optim[last_i];

          // Compute orientation of last
          if (i < (int)path_optim.size()) {
            auto &current = path_optim[i];
            Eigen::Vector2d tangent_dir = tangentDir<double>(
              prelast.block<2, 1>(0, 0),
              last.block<2, 1>(0, 0),
              current.block<2, 1>(0, 0),
              prelast[2]*last[2]);
            
            last_dir = 
              tangent_dir.dot((current - last).block<2, 1>(0, 0)*last[2]) >= 0
                ? tangent_dir
                : -tangent_dir;
            last_dir.normalize();
          }
          else {
            last_dir = end_dir;
          }
          double last_angle = atan2(last_dir[1], last_dir[0]);

          // Interpolate poses between prelast and last
          int interp_cnt = (last_i - prelast_i)*params.output_upsampling_factor - 1;
          if (interp_cnt > 0) {
            Eigen::Vector2d lastp = last.block<2, 1>(0, 0);
            Eigen::Vector2d prelastp = prelast.block<2, 1>(0, 0);
            double dist = (lastp - prelastp).norm();
            Eigen::Vector2d p1 = prelastp + prelast_dir*dist*0.4*prelast[2];
            Eigen::Vector2d p2 = lastp - last_dir*dist*0.4*prelast[2];
            for (int j = 1; j <= interp_cnt; j++) {
              double interp = j/(double)(interp_cnt + 1);
              Eigen::Vector2d p = cubicBezier(prelastp, p1, p2, lastp, interp);
              path.emplace_back(p[0], p[1], 0.0);
            }
          }
          path.emplace_back(last[0], last[1], last_angle);

          // Assign orientations to interpolated points
          for (int j = path.size() - 1 - interp_cnt; j < (int)path.size() - 1; j++) {
            Eigen::Vector2d tangent_dir = tangentDir<double>(
              path[j-1].block<2, 1>(0, 0),
              path[j].block<2, 1>(0, 0),
              path[j+1].block<2, 1>(0, 0),
              1.0);
            tangent_dir = 
              tangent_dir.dot((path[j+1] - path[j]).block<2, 1>(0, 0)*prelast[2]) >= 0
                ? tangent_dir
                : -tangent_dir;
            path[j][2] = atan2(tangent_dir[1], tangent_dir[0]);
          }

          prelast_dir = last_dir;
        }
        prelast_i = last_i;
        last_i = i;
      }
    }
  }

  /*
    Piecewise cubic bezier curve as defined by Adobe in Postscript
    The two end points are p0 and p3
    Their associated control points are p1 and p2
  */
  static Eigen::Vector2d cubicBezier(Eigen::Vector2d &p0, Eigen::Vector2d &p1, Eigen::Vector2d &p2, Eigen::Vector2d &p3, double mu)
  {
    Eigen::Vector2d a,b,c,p;

    c[0] = 3 * (p1[0] - p0[0]);
    c[1] = 3 * (p1[1] - p0[1]);
    b[0] = 3 * (p2[0] - p1[0]) - c[0];
    b[1] = 3 * (p2[1] - p1[1]) - c[1];
    a[0] = p3[0] - p0[0] - c[0] - b[0];
    a[1] = p3[1] - p0[1] - c[1] - b[1];

    p[0] = a[0] * mu * mu * mu + b[0] * mu * mu + c[0] * mu + p0[0];
    p[1] = a[1] * mu * mu * mu + b[1] * mu * mu + c[1] * mu + p0[1];

    return(p);
  }

  bool _debug;
  ceres::Solver::Options _options;
  std::shared_ptr<ceres::Grid2D<u_char>> _costmap_grid;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_CERES_COSTAWARE_SMOOTHER__SMOOTHER_HPP_
