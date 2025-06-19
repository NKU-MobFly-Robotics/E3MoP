/******************************************************************************
 * Copyright (c) 2022, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "ceres_path_smoother/ceres_path_smoother.h"

#include "glog/logging.h"

#include "ceres_path_smoother/cost_functor/deviation_cost_functor.h"
#include "ceres_path_smoother/cost_functor/obstacle_cost_functor.h"
#include "ceres_path_smoother/cost_functor/smooth_cost_functor.h"

namespace ceres_path_smoother {

bool CeresPathSmoother::optimize(
    const std::vector<geometry_msgs::PoseStamped>& initial_path,
    const path_smoother_base::EuclidDistGrid& grid,
    const double inflation_radius_m, std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  // Sanity Check
  if (initial_path.empty()) {
    LOG(ERROR) << "reference points empty, solver early terminates.";
    return false;
  }

  if (initial_path.size() < 3) {
    LOG(ERROR) << "ref_points size smaller than 3, solver early terminates.";
    return false;
  }

  if (initial_path.size() > std::numeric_limits<size_t>::max()) {
    LOG(ERROR) << "ref_points size too large, solver early terminates.";
    return false;
  }

  opt_x->resize(initial_path.size(), 0.0);
  opt_y->resize(initial_path.size(), 0.0);
  for (size_t i = 0; i < initial_path.size(); ++i) {
    opt_x->at(i) = initial_path[i].pose.position.x;
    opt_y->at(i) = initial_path[i].pose.position.y;
  }

  // build a problem
  ceres::Problem problem;

  // add smoothness constraints
  addSmoothnessConstraints(opt_x, opt_y, &problem);

  // add deviation constraints
  addDeviationConstraints(opt_x, opt_y, &problem);

  // add obstacle constraints
  addObstacleConstraints(opt_x, opt_y, grid, inflation_radius_m, &problem);

  // fix the first and last vertices
  problem.SetParameterBlockConstant(&opt_x->front());
  problem.SetParameterBlockConstant(&opt_y->front());
  problem.SetParameterBlockConstant(&opt_x->back());
  problem.SetParameterBlockConstant(&opt_y->back());

  // solve problem
  if (!solveOptimizationProblem(&problem)) {
    return false;
  }
  return true;
}

void CeresPathSmoother::addSmoothnessConstraints(std::vector<double>* opt_x,
                                                 std::vector<double>* opt_y,
                                                 ceres::Problem* problem) {
  CHECK_EQ(opt_x->size(), opt_y->size());
  CHECK_GT(opt_x->size(), 2U);
  ceres::LossFunction* loss_function = nullptr;
  Eigen::Matrix2d information;
  // information 平滑项的权重
  information << 100.0, 0, 0, 100.0;

  for (size_t i = 0; i < opt_x->size() - 2; ++i) {
    // smoothness constraints
    ceres::CostFunction* smoothness_cost_function =
        SmoothCostFunctor::Create(information);
    problem->AddResidualBlock(smoothness_cost_function, loss_function,
                              &opt_x->at(i), &opt_y->at(i), &opt_x->at(i + 1),
                              &opt_y->at(i + 1), &opt_x->at(i + 2),
                              &opt_y->at(i + 2));
  }
}

void CeresPathSmoother::addDeviationConstraints(std::vector<double>* opt_x,
                                                std::vector<double>* opt_y,
                                                ceres::Problem* problem) {
  CHECK_EQ(opt_x->size(), opt_y->size());
  ceres::LossFunction* loss_function = nullptr;
  Eigen::Matrix2d information;
  information << 0.1, 0, 0, 0.1;
  Eigen::Vector2d ref_point;

  for (size_t i = 0; i < opt_x->size(); ++i) {
    ref_point << opt_x->at(i), opt_y->at(i);
    ceres::CostFunction* deviation_cost_function =
        DeviationCostFunctor::Create(information, ref_point);
    problem->AddResidualBlock(deviation_cost_function, loss_function,
                              &opt_x->at(i), &opt_y->at(i));
  }
}

void CeresPathSmoother::addObstacleConstraints(
    std::vector<double>* opt_x, std::vector<double>* opt_y,
    const path_smoother_base::EuclidDistGrid& grid,
    const double inflation_radius_m, ceres::Problem* problem) {
  CHECK_EQ(opt_x->size(), opt_y->size());
  ceres::LossFunction* loss_function = nullptr;
  Eigen::Matrix<double, 1, 1> information;
  information << 1.0;

  for (size_t i = 0; i < opt_x->size(); ++i) {
    ceres::CostFunction* obstacle_cost_function =
        ObstacleCostFunctor::Create(grid, inflation_radius_m, information);
    problem->AddResidualBlock(obstacle_cost_function, loss_function,
                              &opt_x->at(i), &opt_y->at(i));
  }
}

bool CeresPathSmoother::solveOptimizationProblem(ceres::Problem* problem) {
  // solve the problem
  ceres::Solver::Options options;
  options.max_num_iterations = 500;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  std::cout << summary.BriefReport() << "\n";

  return summary.IsSolutionUsable();
}

}  // namespace ceres_path_smoother
