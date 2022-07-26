/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file piecewise_jerk_path_optimizer.cc
 **/

#include "modules/planning/tasks/optimizers/piecewise_jerk_path/piecewise_jerk_path_optimizer.h"

#include <memory>
#include <string>

#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/trajectory1d/piecewise_jerk_trajectory1d.h"
#include "modules/planning/math/piecewise_jerk/piecewise_jerk_path_problem.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Gaussian;

// 在工厂类task_factory.cc中新建该类
PiecewiseJerkPathOptimizer::PiecewiseJerkPathOptimizer(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : PathOptimizer(config, injector) {
  ACHECK(config_.has_piecewise_jerk_path_optimizer_config());
}

// 在path_optimizer.cc中被调用
common::Status PiecewiseJerkPathOptimizer::Process(
    const SpeedData& speed_data, const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point, const bool path_reusable,
    PathData* const final_path_data) {
  // skip piecewise_jerk_path_optimizer if reused path
  if (FLAGS_enable_skip_path_tasks && path_reusable) {
    return Status::OK();
  }
  // 路径起始点的位置朝向，起始点就是车辆后方的某点
  // TODO(yxb): 起始点寻找方法需要再看一遍
  ADEBUG << "Plan at the starting point: x = " << init_point.path_point().x()
         << ", y = " << init_point.path_point().y()
         << ", and angle = " << init_point.path_point().theta();
  common::TrajectoryPoint planning_start_point = init_point;
  if (FLAGS_use_front_axe_center_in_path_planning) {
    planning_start_point =
        InferFrontAxeCenterFromRearAxeCenter(planning_start_point);
  }
  // 定义一个对组的数据结构，对组中是两个数组，每个数组的维数是3
  // 得到s，s点，s两点； l，l撇，l两撇
  const auto init_frenet_state =
      reference_line.ToFrenetFrame(planning_start_point);

  // Choose lane_change_path_config for lane-change cases
  // Otherwise, choose default_path_config for normal path planning
  // 换道有自己特殊的一组优化参数
  // config中有l，l撇，l两撇，l三撇的权重
  const auto& config = reference_line_info_->IsChangeLanePath()
                           ? config_.piecewise_jerk_path_optimizer_config()
                                 .lane_change_path_config()
                           : config_.piecewise_jerk_path_optimizer_config()
                                 .default_path_config();

  // l，dl，ddl，dddl的权重在这里读取
  // apollo在构造cost函数和将cost函数转换为QP问题时，并不是一一对应的
  // cost中有l，l撇，l两撇，l三撇四项
  // QP中将四项合并为l，l撇，l两撇三项，但是产生了非平方的二次项
  // 所以QP中优化变量的约束取法，并不在限定在l，l撇，l两撇，l三撇，而是从它们的物理意义出发
  // 将它们转换为tan（heading）和kappa的函数，因此在施加变量约束的时候，是在给tan（heading）
  // 和kappa施加。所以以下w中dl权重出现了vs^2，而ddl和dddl的权重中虽然没有vs的多次项，
  // 但从它们初始化时的数量级来看，已经暗含了vs的2次项或4次项
  std::array<double, 5> w = {
      config.l_weight(),
      config.dl_weight() *
          std::fmax(init_frenet_state.first[1] * init_frenet_state.first[1],
                    5.0),
      config.ddl_weight(), config.dddl_weight(), 0.0};

  const auto& path_boundaries =
      reference_line_info_->GetCandidatePathBoundaries();
  // 这个大小表示boundary的vector中包含的path boundary对象的个数
  ADEBUG << "There are " << path_boundaries.size() << " path boundaries.";
  const auto& reference_path_data = reference_line_info_->path_data();

  std::vector<PathData> candidate_path_data;
  for (const auto& path_boundary : path_boundaries) {
    size_t path_boundary_size = path_boundary.boundary().size();

    // if the path_boundary is normal, it is possible to have less than 2 points
    // skip path boundary of this kind

    // continue强迫跳出当前循环，执行到下一个循环
    // find函数的返回值是整数，假如字符串存在包含关系，其返回值必定不等于npos，
    // 但如果字符串不存在包含关系，那么返回值就一定是npos

    // 就是说如果边界的标签是regular的并且只有两个点，那么跳到下一次循环
    if (path_boundary.label().find("regular") != std::string::npos &&
        path_boundary_size < 2) {
      continue;
    }

    int max_iter = 4000;
    // lower max_iter for regular/self/

    // 假如路径边界的标签中存在self，即是直行或借道，那么最大迭代步给4000
    if (path_boundary.label().find("self") != std::string::npos) {
      max_iter = 4000;
    }

    CHECK_GT(path_boundary_size, 1);

    std::vector<double> opt_l;
    std::vector<double> opt_dl;
    std::vector<double> opt_ddl;

    std::array<double, 3> end_state = {0.0, 0.0, 0.0};

    if (!FLAGS_enable_force_pull_over_open_space_parking_test) {
      // pull over scenario
      // set end lateral to be at the desired pull over destination
      const auto& pull_over_status =
          injector_->planning_context()->planning_status().pull_over();
      if (pull_over_status.has_position() &&
          pull_over_status.position().has_x() &&
          pull_over_status.position().has_y() &&
          path_boundary.label().find("pullover") != std::string::npos) {
        common::SLPoint pull_over_sl;
        reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
        end_state[0] = pull_over_sl.l();
      }
    }

    // TODO(all): double-check this;
    // final_path_data might carry info from upper stream

    PathData path_data = *final_path_data;

    // updated cost function for path reference
    std::vector<double> path_reference_l(path_boundary_size, 0.0);
    bool is_valid_path_reference = false;
    size_t path_reference_size = reference_path_data.path_reference().size();

    // 只要不是fallback的参考线，都会带有regular
    // 所以不论是本车道，绕行，换道，都会把l方向的边界赋给path_reference_l
    if (path_boundary.label().find("regular") != std::string::npos &&
        reference_path_data.is_valid_path_reference()) {
      ADEBUG << "path label is: " << path_boundary.label();
      // when path reference is ready
      for (size_t i = 0; i < path_reference_size; ++i) {
        common::SLPoint path_reference_sl;
        // 把参考线中离散点的xy全部转换为sl
        reference_line.XYToSL(
            common::util::PointFactory::ToPointENU(
                reference_path_data.path_reference().at(i).x(),
                reference_path_data.path_reference().at(i).y()),
            &path_reference_sl);
        path_reference_l[i] = path_reference_sl.l();
      }
      // 把参考线最后一个点作为终点状态
      end_state[0] = path_reference_l.back();
      path_data.set_is_optimized_towards_trajectory_reference(true);
      is_valid_path_reference = true;
    }

    const auto& veh_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    // 这是曲率，不是加速度，符号起名不好
    const double lat_acc_bound =
        std::tan(veh_param.max_steer_angle() / veh_param.steer_ratio()) /
        veh_param.wheel_base();
    std::vector<std::pair<double, double>> ddl_bounds;
    for (size_t i = 0; i < path_boundary_size; ++i) {
      // 使用static_cast可以明确告诉编译器，这种损失精度的转换是在知情的情况下进行的
      // 也可以让阅读程序的其他程序员明确你转换的目的而不是由于疏忽
      // 计算某个参考线上离散点的s值
      double s = static_cast<double>(i) * path_boundary.delta_s() +
                 path_boundary.start_s();
      // 用s值去查该点的曲率
      double kappa = reference_line.GetNearestReferencePoint(s).kappa();
      // 参考线的曲率和车辆能够运行最大曲率之间的差，就是车辆在参考线的基础上，如果采用了最大的转角
      // 能运行出来的曲线在这个离散点上的曲率区间
      // 为什么要把曲率塞到ddl的约束里？这里实际上限制的QP的优化变量就已经是曲率了
      // 在cost函数转化为QP的优化函数时，把原l两撇转换为速度vs与曲率的函数了
      ddl_bounds.emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
    }

    // 路径优化的主函数
    // 输入的end_state只有l有值，l撇，l两撇都为0
    // 为什么在cost函数中没有l三撇平方项，是因为该项是常数项吗？
    // 那非状态终端的这些离散点的l三撇是常数，它的平方项为什么不略去？
    bool res_opt = OptimizePath(
        init_frenet_state.second, end_state, std::move(path_reference_l),
        path_reference_size, path_boundary.delta_s(), is_valid_path_reference,
        path_boundary.boundary(), ddl_bounds, w, max_iter, &opt_l, &opt_dl,
        &opt_ddl);

    if (res_opt) {
      for (size_t i = 0; i < path_boundary_size; i += 4) {
        ADEBUG << "for s[" << static_cast<double>(i) * path_boundary.delta_s()
               << "], l = " << opt_l[i] << ", dl = " << opt_dl[i];
      }
      auto frenet_frame_path =
          ToPiecewiseJerkPath(opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
                              path_boundary.start_s());

      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(std::move(frenet_frame_path));
      if (FLAGS_use_front_axe_center_in_path_planning) {
        auto discretized_path = DiscretizedPath(
            ConvertPathPointRefFromFrontAxeToRearAxe(path_data));
        path_data.SetDiscretizedPath(discretized_path);
      }
      path_data.set_path_label(path_boundary.label());
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
      candidate_path_data.push_back(std::move(path_data));
    }
  }
  if (candidate_path_data.empty()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Path Optimizer failed to generate path");
  }
  reference_line_info_->SetCandidatePathData(std::move(candidate_path_data));
  return Status::OK();
}

common::TrajectoryPoint
PiecewiseJerkPathOptimizer::InferFrontAxeCenterFromRearAxeCenter(
    const common::TrajectoryPoint& traj_point) {
  double front_to_rear_axe_distance =
      VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();
  common::TrajectoryPoint ret = traj_point;
  ret.mutable_path_point()->set_x(
      traj_point.path_point().x() +
      front_to_rear_axe_distance * std::cos(traj_point.path_point().theta()));
  ret.mutable_path_point()->set_y(
      traj_point.path_point().y() +
      front_to_rear_axe_distance * std::sin(traj_point.path_point().theta()));
  return ret;
}

std::vector<common::PathPoint>
PiecewiseJerkPathOptimizer::ConvertPathPointRefFromFrontAxeToRearAxe(
    const PathData& path_data) {
  std::vector<common::PathPoint> ret;
  double front_to_rear_axe_distance =
      VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();
  for (auto path_point : path_data.discretized_path()) {
    common::PathPoint new_path_point = path_point;
    new_path_point.set_x(path_point.x() - front_to_rear_axe_distance *
                                              std::cos(path_point.theta()));
    new_path_point.set_y(path_point.y() - front_to_rear_axe_distance *
                                              std::sin(path_point.theta()));
    ret.push_back(new_path_point);
  }
  return ret;
}

bool PiecewiseJerkPathOptimizer::OptimizePath(
    const std::array<double, 3>& init_state,
    const std::array<double, 3>& end_state,
    std::vector<double> path_reference_l_ref, const size_t path_reference_size,
    const double delta_s, const bool is_valid_path_reference,
    const std::vector<std::pair<double, double>>& lat_boundaries,
    const std::vector<std::pair<double, double>>& ddl_bounds,
    const std::array<double, 5>& w, const int max_iter, std::vector<double>* x,
    std::vector<double>* dx, std::vector<double>* ddx) {
  // num of knots
  const size_t kNumKnots = lat_boundaries.size();
  // 用类构造了一个对象
  PiecewiseJerkPathProblem piecewise_jerk_problem(kNumKnots, delta_s,
                                                  init_state);

  // TODO(Hongyi): update end_state settings
  // 这里直接赋值终点的权值
  piecewise_jerk_problem.set_end_state_ref({1000.0, 0.0, 0.0}, end_state);
  // pull over scenarios

  // 参考路径使终端状态不为零，这难道不是常态吗？还是说终端状态为0是常态？
  // 终端状态不为零，表示这一系列点都跑完了，但是车辆与参考线之间还是有横向距离以及几阶导
  // 这其实是很常见的，尤其是参考线最后一个点并不是全局路径终点时经常出现

  // Because path reference might also make the end_state != 0
  // we have to exclude this condition here
  // 紧急靠边停车，增加特殊判定
  if (end_state[0] != 0 && !is_valid_path_reference) {
    // 为什么要给（二次规划中）每个点都添加终点状态的L值？
    std::vector<double> x_ref(kNumKnots, end_state[0]);
    const auto& pull_over_type = injector_->planning_context()
                                     ->planning_status()
                                     .pull_over()
                                     .pull_over_type();
    // 终端状态中l的权重
    const double weight_x_ref =
        pull_over_type == PullOverStatus::EMERGENCY_PULL_OVER ? 200.0 : 10.0;
    piecewise_jerk_problem.set_x_ref(weight_x_ref, std::move(x_ref));
  }
  // use path reference as a optimization cost function
  if (is_valid_path_reference) {
    // for non-path-reference part
    // weight_x_ref is set to default value, where
    // l weight = weight_x_ + weight_x_ref_ = (1.0 + 0.0)
    // 以下全部不能理解
    std::vector<double> weight_x_ref_vec(kNumKnots, 0.0);
    // increase l weight for path reference part only

    // path_reference_l_weight表示，在存在（机器或强化）学习模型时，学习模型对优化的影响
    // 这里的peak有何具体含义？
    const double peak_value = config_.piecewise_jerk_path_optimizer_config()
                                  .path_reference_l_weight();
    // 0.5 * 点的个数 * deltas，什么含义？
    const double peak_value_x =
        0.5 * static_cast<double>(path_reference_size) * delta_s;
    for (size_t i = 0; i < path_reference_size; ++i) {
      // Gaussian weighting
      const double x = static_cast<double>(i) * delta_s;
      weight_x_ref_vec.at(i) = GaussianWeighting(x, peak_value, peak_value_x);
      ADEBUG << "i: " << i << ", weight: " << weight_x_ref_vec.at(i);
    }
    piecewise_jerk_problem.set_x_ref(std::move(weight_x_ref_vec),
                                     std::move(path_reference_l_ref));
  }

  piecewise_jerk_problem.set_weight_x(w[0]);
  piecewise_jerk_problem.set_weight_dx(w[1]);
  piecewise_jerk_problem.set_weight_ddx(w[2]);
  piecewise_jerk_problem.set_weight_dddx(w[3]);

  // 缩放因子的设置依赖什么？
  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});

  auto start_time = std::chrono::system_clock::now();

  piecewise_jerk_problem.set_x_bounds(lat_boundaries);
  // dx的约束，实际上是tan(heading)的约束
  // 这里的heading在xy投影到sl坐标系时，就已经变成当前车辆相对参考线航向的角度差了
  // 所以取上下限63度是完全足够的
  piecewise_jerk_problem.set_dx_bounds(-FLAGS_lateral_derivative_bound_default,
                                       FLAGS_lateral_derivative_bound_default);
  piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);

  // Estimate lat_acc and jerk boundary from vehicle_params
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double axis_distance = veh_param.wheel_base();
  // 为什么要除2？
  // 这里计算的是前轮转角的变化率!!!不是横摆角的变化率，这两者有差异！
  const double max_yaw_rate =
      veh_param.max_steer_angle_rate() / veh_param.steer_ratio() / 2.0;
  // a_l=vs^2/R=k*vs^2
  // jerk=delta(a_l)/delta(s)=delta(kappa)/delta(s)*vs^2
  // tan(前轮转角)=L*kappa，近似，前轮转角=L*kappa，delta(前轮转角)=L*delta(kappa)
  // 则jerk=delta(前轮转角)/( L*delta(s) ) * vs^2 
  // 所以，代码中的vehicle_speed是不是应该是纵向速度？
  const double jerk_bound = EstimateJerkBoundary(std::fmax(init_state[1], 1.0),
                                                 axis_distance, max_yaw_rate);
  // ? 与当前demo中的planning代码进行对比 ？
  // 乘以delta_s的操作在更下层代码中
  piecewise_jerk_problem.set_dddx_bound(jerk_bound);

  // 调用piecewise_jerk_problem.cc中的PiecewiseJerkProblem::Optimize
  bool success = piecewise_jerk_problem.Optimize(max_iter);

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  ADEBUG << "Path Optimizer used time: " << diff.count() * 1000 << " ms.";

  if (!success) {
    AERROR << "piecewise jerk path optimizer failed";
    return false;
  }

  *x = piecewise_jerk_problem.opt_x();
  *dx = piecewise_jerk_problem.opt_dx();
  *ddx = piecewise_jerk_problem.opt_ddx();

  return true;
}

FrenetFramePath PiecewiseJerkPathOptimizer::ToPiecewiseJerkPath(
    const std::vector<double>& x, const std::vector<double>& dx,
    const std::vector<double>& ddx, const double delta_s,
    const double start_s) const {
  ACHECK(!x.empty());
  ACHECK(!dx.empty());
  ACHECK(!ddx.empty());

  PiecewiseJerkTrajectory1d piecewise_jerk_traj(x.front(), dx.front(),
                                                ddx.front());

  for (std::size_t i = 1; i < x.size(); ++i) {
    const auto dddl = (ddx[i] - ddx[i - 1]) / delta_s;
    piecewise_jerk_traj.AppendSegment(dddl, delta_s);
  }

  std::vector<common::FrenetFramePoint> frenet_frame_path;
  double accumulated_s = 0.0;
  while (accumulated_s < piecewise_jerk_traj.ParamLength()) {
    double l = piecewise_jerk_traj.Evaluate(0, accumulated_s);
    double dl = piecewise_jerk_traj.Evaluate(1, accumulated_s);
    double ddl = piecewise_jerk_traj.Evaluate(2, accumulated_s);

    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(accumulated_s + start_s);
    frenet_frame_point.set_l(l);
    frenet_frame_point.set_dl(dl);
    frenet_frame_point.set_ddl(ddl);
    frenet_frame_path.push_back(std::move(frenet_frame_point));

    accumulated_s += FLAGS_trajectory_space_resolution;
  }

  return FrenetFramePath(std::move(frenet_frame_path));
}

double PiecewiseJerkPathOptimizer::EstimateJerkBoundary(
    const double vehicle_speed, const double axis_distance,
    const double max_yaw_rate) const {
  return max_yaw_rate / axis_distance / vehicle_speed;
}

double PiecewiseJerkPathOptimizer::GaussianWeighting(
    const double x, const double peak_weighting,
    const double peak_weighting_x) const {
  double std = 1 / (std::sqrt(2 * M_PI) * peak_weighting);
  double u = peak_weighting_x * std;
  double x_updated = x * std;
  ADEBUG << peak_weighting *
                exp(-0.5 * (x - peak_weighting_x) * (x - peak_weighting_x));
  ADEBUG << Gaussian(u, std, x_updated);
  return Gaussian(u, std, x_updated);
}

}  // namespace planning
}  // namespace apollo
