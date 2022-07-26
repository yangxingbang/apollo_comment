/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/control/controller/mpc_controller.h"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "absl/strings/str_cat.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/mpc_osqp.h"
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace control {
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleStateProvider;
using apollo::cyber::Clock;
using Matrix = Eigen::MatrixXd;
using apollo::common::VehicleConfigHelper;

// 处理log的函数自己单独组成了一个namespace，mpc的函数不在这个namespace里
namespace {

std::string GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);

  std::tm time_tm;
  localtime_r(&raw_time, &time_tm);
  strftime(name_buffer, sizeof(name_buffer),
           "/tmp/mpc_controller_%F_%H%M%S.csv", &time_tm);
  return std::string(name_buffer);
}

void WriteHeaders(std::ofstream &file_stream) {}
}  // namespace

MPCController::MPCController() : name_("MPC Controller") {
  // FLAGS_enable_csv_debug默认为true
  if (FLAGS_enable_csv_debug) {
    mpc_log_file_.open(GetLogFileName());
    mpc_log_file_ << std::fixed;
    mpc_log_file_ << std::setprecision(6);
    WriteHeaders(mpc_log_file_);
  }
  AINFO << "Using " << name_;
}

MPCController::~MPCController() { CloseLogFile(); }

bool MPCController::LoadControlConf(const ControlConf *control_conf) {
  if (!control_conf) {
    AERROR << "[MPCController] control_conf = nullptr";
    return false;
  }
  // 读取车辆参数
  vehicle_param_ = VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  // 读取control_conf.pb.txt中mpc_controller_conf一项中的ts
  ts_ = control_conf->mpc_controller_conf().ts();
  if (ts_ <= 0.0) {
    AERROR << "[MPCController] Invalid control update interval.";
    return false;
  }
  cf_ = control_conf->mpc_controller_conf().cf();
  cr_ = control_conf->mpc_controller_conf().cr();
  wheelbase_ = vehicle_param_.wheel_base();
  steer_ratio_ = vehicle_param_.steer_ratio();
  // 方向盘转角的最大值，角度
  steer_single_direction_max_degree_ =
      vehicle_param_.max_steer_angle() * 180 / M_PI;
  
  // lincolnMKZ默认5m/s2,为0.5g
  max_lat_acc_ = control_conf->mpc_controller_conf().max_lateral_acceleration();

  // TODO(Shu, Qi, Yu): add sanity check for conf values
  // steering ratio should be positive
  static constexpr double kEpsilon = 1e-6;
  // 检查转向传动比是否有错
  if (std::isnan(steer_ratio_) || steer_ratio_ < kEpsilon) {
    AERROR << "[MPCController] steer_ratio = 0";
    return false;
  }
  // 车轮最大转角，角度
  wheel_single_direction_max_degree_ =
      steer_single_direction_max_degree_ / steer_ratio_ / 180 * M_PI;
  // 2m/s2
  max_acceleration_ = vehicle_param_.max_acceleration();
  // -6m/s2
  max_deceleration_ = vehicle_param_.max_deceleration();

  // 520，520，520，520
  const double mass_fl = control_conf->mpc_controller_conf().mass_fl();
  const double mass_fr = control_conf->mpc_controller_conf().mass_fr();
  const double mass_rl = control_conf->mpc_controller_conf().mass_rl();
  const double mass_rr = control_conf->mpc_controller_conf().mass_rr();
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;

  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  // 0.01,mpc最优化求解器的误差限
  mpc_eps_ = control_conf->mpc_controller_conf().eps();
  // 150
  mpc_max_iteration_ = control_conf->mpc_controller_conf().max_iteration();
  // mkz油门死区15.7% 
  throttle_lowerbound_ =
      std::max(vehicle_param_.throttle_deadzone(),
               control_conf->mpc_controller_conf().throttle_minimum_action());
  // mkz制动死区15.7% 
  brake_lowerbound_ =
      std::max(vehicle_param_.brake_deadzone(),
               control_conf->mpc_controller_conf().brake_minimum_action());

  // 0.1m/s
  minimum_speed_protection_ = control_conf->minimum_speed_protection();
  // 0.01m/s2
  max_acceleration_when_stopped_ =
      control_conf->max_acceleration_when_stopped();
  // 0.2m/s
  max_abs_speed_when_stopped_ = vehicle_param_.max_abs_speed_when_stopped();
  standstill_acceleration_ =
      control_conf->mpc_controller_conf().standstill_acceleration();

  enable_mpc_feedforward_compensation_ =
      control_conf->mpc_controller_conf().enable_mpc_feedforward_compensation();

  // 非约束的控制变量差的极限
  unconstrained_control_diff_limit_ =
      control_conf->mpc_controller_conf().unconstrained_control_diff_limit();

  LoadControlCalibrationTable(control_conf->mpc_controller_conf());
  ADEBUG << "MPC conf loaded";
  return true;
}

void MPCController::ProcessLogs(const SimpleMPCDebug *debug,
                                const canbus::Chassis *chassis) {
  // TODO(QiL): Add debug information
}

void MPCController::LogInitParameters() {
  ADEBUG << name_ << " begin.";
  ADEBUG << "[MPCController parameters]"
         << " mass_: " << mass_ << ","
         << " iz_: " << iz_ << ","
         << " lf_: " << lf_ << ","
         << " lr_: " << lr_;
}

void MPCController::InitializeFilters(const ControlConf *control_conf) {
  // Low pass filter
  // 创建3个元素的vector，都赋值0
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  // 求出二阶滤波器的系数
  common::LpfCoefficients(
      ts_, control_conf->mpc_controller_conf().cutoff_freq(), &den, &num);
  // 设定二阶滤波器的系数
  digital_filter_.set_coefficients(den, num);
  // 构建一个横向误差和航向误差的均值滤波器的对象，滤波窗口10个，对10个数均值滤波
  lateral_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->mpc_controller_conf().mean_filter_window_size()));
  heading_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->mpc_controller_conf().mean_filter_window_size()));
}

Status MPCController::Init(std::shared_ptr<DependencyInjector> injector,
                           const ControlConf *control_conf) {
  // 传递的依然是control_conf.pb.txt
  if (!LoadControlConf(control_conf)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");
  }
  injector_ = injector;
  // Matrix init operations.
  // 6*6的A矩阵
  // TODO(yxb): 所有A，B，C中的Cf，Cr都需要乘以2
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  // TODO(yxb): 所有离散化矩阵都需要重新计算
  matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  // 结合A、B、C矩阵，我们发现纵向mpc控制中使用的模型是：单点质量运行学模型
  matrix_a_(4, 5) = 1.0;
  matrix_a_(5, 5) = 0.0;
  // TODO(QiL): expand the model to accommodate more combined states.

  matrix_a_coeff_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(2, 3) = 1.0;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  // 6行2列
  matrix_b_ = Matrix::Zero(basic_state_size_, controls_);
  matrix_bd_ = Matrix::Zero(basic_state_size_, controls_);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_b_(4, 1) = 0.0;
  matrix_b_(5, 1) = -1.0;
  // 有错！
  matrix_bd_ = matrix_b_ * ts_;

  matrix_c_ = Matrix::Zero(basic_state_size_, 1);
  matrix_cd_ = Matrix::Zero(basic_state_size_, 1);

  matrix_state_ = Matrix::Zero(basic_state_size_, 1);
  // 1行6列，没有用到
  matrix_k_ = Matrix::Zero(1, basic_state_size_);

  // R是两行两列的单位矩阵
  matrix_r_ = Matrix::Identity(controls_, controls_);

  // Q是6行6列的零矩阵
  matrix_q_ = Matrix::Zero(basic_state_size_, basic_state_size_);

  // 默认的R的对角元素是3.25，1.0；共2个元素
  // 注意r_param_size是搜不到的，它是proto中某个repeated变量的计数
  int r_param_size = control_conf->mpc_controller_conf().matrix_r_size();
  for (int i = 0; i < r_param_size; ++i) {
    matrix_r_(i, i) = control_conf->mpc_controller_conf().matrix_r(i);
  }

  // 默认的Q对角元素是3.0，0.0，35，0.0，50，10；共6个元素
  int q_param_size = control_conf->mpc_controller_conf().matrix_q_size();
  if (basic_state_size_ != q_param_size) {
    const auto error_msg =
        absl::StrCat("MPC controller error: matrix_q size: ", q_param_size,
                     " in parameter file not equal to basic_state_size_: ",
                     basic_state_size_);
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }
  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf->mpc_controller_conf().matrix_q(i);
  }

  // Update matrix_q_updated_ and matrix_r_updated_
  matrix_r_updated_ = matrix_r_;
  matrix_q_updated_ = matrix_q_;

  // 初始化滤波器，包括计算二阶滤波器的分子分母系数，然后设定这些系数
  InitializeFilters(control_conf);
  LoadMPCGainScheduler(control_conf->mpc_controller_conf());
  LogInitParameters();
  ADEBUG << "[MPCController] init done!";
  return Status::OK();
}

void MPCController::CloseLogFile() {
  if (FLAGS_enable_csv_debug && mpc_log_file_.is_open()) {
    mpc_log_file_.close();
  }
}

// 车轮转角转化为方向盘转角百分比
double MPCController::Wheel2SteerPct(const double wheel_angle) {
  return wheel_angle / wheel_single_direction_max_degree_ * 100;
}

void MPCController::Stop() { CloseLogFile(); }

std::string MPCController::Name() const { return name_; }

void MPCController::LoadMPCGainScheduler(
    const MPCControllerConf &mpc_controller_conf) {
  // 为什么只有这四项增加调度增益
  // 横向误差，航向误差，前馈项，方向盘转角权项的调度增益
  const auto &lat_err_gain_scheduler =
      mpc_controller_conf.lat_err_gain_scheduler();
  const auto &heading_err_gain_scheduler =
      mpc_controller_conf.heading_err_gain_scheduler();
  const auto &feedforwardterm_gain_scheduler =
      mpc_controller_conf.feedforwardterm_gain_scheduler();
  const auto &steer_weight_gain_scheduler =
      mpc_controller_conf.steer_weight_gain_scheduler();
  ADEBUG << "MPC control gain scheduler loaded";
  Interpolation1D::DataType xy1, xy2, xy3, xy4;
  // 横向误差的调度增益一共有6项，（6个车速，6个增益值）
  for (const auto &scheduler : lat_err_gain_scheduler.scheduler()) {
    xy1.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  // 航向误差的调度增益一共有6项，（6个车速，6个增益值）
  for (const auto &scheduler : heading_err_gain_scheduler.scheduler()) {
    xy2.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  // 前馈项的调度增益一共有6项，（6个车速，6个增益值）
  for (const auto &scheduler : feedforwardterm_gain_scheduler.scheduler()) {
    xy3.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  // 方向盘转角权项的调度增益
  for (const auto &scheduler : steer_weight_gain_scheduler.scheduler()) {
    xy4.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }

  lat_err_interpolation_.reset(new Interpolation1D);
  // 有了6组车速和增益，形成一个表，然后对表进行多项式拟合，然后插值，插值后的结果放在指针对应的内存中
  ACHECK(lat_err_interpolation_->Init(xy1))
      << "Fail to load lateral error gain scheduler for MPC controller";

  heading_err_interpolation_.reset(new Interpolation1D);
  ACHECK(heading_err_interpolation_->Init(xy2))
      << "Fail to load heading error gain scheduler for MPC controller";

  feedforwardterm_interpolation_.reset(new Interpolation1D);
  ACHECK(feedforwardterm_interpolation_->Init(xy3))
      << "Fail to load feed forward term gain scheduler for MPC controller";

  steer_weight_interpolation_.reset(new Interpolation1D);
  ACHECK(steer_weight_interpolation_->Init(xy4))
      << "Fail to load steer weight gain scheduler for MPC controller";
}

Status MPCController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    ControlCommand *cmd) {
  trajectory_analyzer_ =
      std::move(TrajectoryAnalyzer(planning_published_trajectory));

  SimpleMPCDebug *debug = cmd->mutable_debug()->mutable_simple_mpc_debug();
  debug->Clear();

  ComputeLongitudinalErrors(&trajectory_analyzer_, debug);

  // Update state
  UpdateState(debug);

  UpdateMatrix(debug);

  FeedforwardUpdate(debug);

  auto vehicle_state = injector_->vehicle_state();
  // Add gain scheduler for higher speed steering
  // FLAGS_enable_gain_scheduler没有声明和定义，enable_gain_scheduler
  if (FLAGS_enable_gain_scheduler) {
    matrix_q_updated_(0, 0) =
        matrix_q_(0, 0) *
        lat_err_interpolation_->Interpolate(vehicle_state->linear_velocity());
    matrix_q_updated_(2, 2) =
        matrix_q_(2, 2) * heading_err_interpolation_->Interpolate(
                              vehicle_state->linear_velocity());
    steer_angle_feedforwardterm_updated_ =
        steer_angle_feedforwardterm_ *
        feedforwardterm_interpolation_->Interpolate(
            vehicle_state->linear_velocity());
    matrix_r_updated_(0, 0) =
        matrix_r_(0, 0) * steer_weight_interpolation_->Interpolate(
                              vehicle_state->linear_velocity());
  } else {
    matrix_q_updated_ = matrix_q_;
    matrix_r_updated_ = matrix_r_;
    steer_angle_feedforwardterm_updated_ = steer_angle_feedforwardterm_;
  }

  // 把Q矩阵中的横向项更新了
  // 对应纵向项的为什么不更新？
  // 如果是因为FLAGS_enable_gain_scheduler的判断分支更新了（0，0）和（2,2）
  // 那（1，1）和（3，3）没在分支中修改，为什么要修改？
  debug->add_matrix_q_updated(matrix_q_updated_(0, 0));
  debug->add_matrix_q_updated(matrix_q_updated_(1, 1));
  debug->add_matrix_q_updated(matrix_q_updated_(2, 2));
  debug->add_matrix_q_updated(matrix_q_updated_(3, 3));

  // 把R矩阵中的横纵向项都更新了
  debug->add_matrix_r_updated(matrix_r_updated_(0, 0));
  debug->add_matrix_r_updated(matrix_r_updated_(1, 1));

  // 控制矩阵，2行1列
  Matrix control_matrix = Matrix::Zero(controls_, 1);
  // 控制序列，序列的长度是预测时域那么长，10个元素，每个元素是一个控制矩阵
  std::vector<Matrix> control(horizon_, control_matrix);

  // 控制增益矩阵，2行6列
  Matrix control_gain_matrix = Matrix::Zero(controls_, basic_state_size_);
  // 控制增益序列，序列的长度是预测时域那么长，10个元素，每个元素是一个控制增益矩阵
  std::vector<Matrix> control_gain(horizon_, control_gain_matrix);

  // 额外增益矩阵，2行1列
  Matrix addition_gain_matrix = Matrix::Zero(controls_, 1);
  // 额外增益序列，序列的长度是预测时域那么长，10个元素，每个元素是一个额外增益矩阵
  std::vector<Matrix> addition_gain(horizon_, addition_gain_matrix);

  // 参考矩阵，6行1列
  Matrix reference_state = Matrix::Zero(basic_state_size_, 1);
  // 参考序列，序列的长度是预测时域那么长，10个元素，每个元素是一个参考矩阵
  std::vector<Matrix> reference(horizon_, reference_state);

  // 角度和加速度的下限，2行1列
  Matrix lower_bound(controls_, 1);
  lower_bound << -wheel_single_direction_max_degree_, max_deceleration_;

  // 角度和加速度的上限，2行1列
  Matrix upper_bound(controls_, 1);
  upper_bound << wheel_single_direction_max_degree_, max_acceleration_;

  const double max = std::numeric_limits<double>::max();
  // 状态变量的下限和上限
  Matrix lower_state_bound(basic_state_size_, 1);
  Matrix upper_state_bound(basic_state_size_, 1);

  // lateral_error, lateral_error_rate, heading_error, heading_error_rate
  // station_error, station_error_rate
  // 航向角误差范围是-180~180度
  lower_state_bound << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max,
      -1.0 * max, -1.0 * max;
  upper_state_bound << max, max, M_PI, max, max, max;

  double mpc_start_timestamp = Clock::NowInSeconds();
  // 反馈转向角
  double steer_angle_feedback = 0.0;
  // 注意这个acc_feedback在定义的时候，它的符号就是与“误差计算”中的加速度误差的符号是相反的！
  // 这样才会出现后文的acceleration_cmd = acc_feedback + debug->acceleration_reference()
  // 这个反馈值是一个补偿量，与串级PID计算出的补偿量是相同性质的
  double acc_feedback = 0.0;
  // 前馈转向角的补偿
  double steer_angle_ff_compensation = 0.0;
  // 无约束的控制量的差
  double unconstrained_control_diff = 0.0;
  // 控制增益截断比
  double control_gain_truncation_ratio = 0.0;
  // 无约束的控制
  double unconstrained_control = 0.0;
  const double v = injector_->vehicle_state()->linear_velocity();

  // 控制命令序列，2个元素，每个元素都是数值
  std::vector<double> control_cmd(controls_, 0);

  // 用MpcOsqp的类的有参构造函数定义了mpc_osqp对象
  // matrix_state_传进去的是X的初始状态矩阵
  // mpc问题中的控制量是 前轮转角 和 加速度补偿量
  // TODO(yxb): 控制量是u还是delta_u，应该看离散后构建的状态方程，需要再核实？
  apollo::common::math::MpcOsqp mpc_osqp(
      matrix_ad_, matrix_bd_, matrix_q_updated_, matrix_r_updated_,
      matrix_state_, lower_bound, upper_bound, lower_state_bound,
      upper_state_bound, reference_state, mpc_max_iteration_, horizon_,
      mpc_eps_);
  // Solve求解出control_cmd，并返回一个布尔状态
  // 最优化过程状态有问题，或者解为空的时候，会返回false
  if (!mpc_osqp.Solve(&control_cmd)) {
    AERROR << "MPC OSQP solver failed";
  } else {
    ADEBUG << "MPC OSQP problem solved! ";
    // vector索引到元素，元素是矩阵，矩阵再索引到元素
    control[0](0, 0) = control_cmd.at(0);
    control[0](1, 0) = control_cmd.at(1);
  }

  // mpc求解得到的反馈方向盘转角
  steer_angle_feedback = Wheel2SteerPct(control[0](0, 0));
  // mpc求解得到的反馈acc
  acc_feedback = control[0](1, 0);

  // 以下是计算steer_angle_ff_compensation

  // 控制增益序列中第一个元素，这个元素是矩阵，该矩阵6行1列
  // 虽然这里对每一个状态量都施加了 增量
  // 但是最后起作用的 增量 只有航向误差那一行
  for (int i = 0; i < basic_state_size_; ++i) {
    unconstrained_control += control_gain[0](0, i) * matrix_state_(i, 0);
  }
  // 把a_g * v / R作为一个 增量，施加到前馈模块
  // 这里R是道路的半径
  unconstrained_control += addition_gain[0](0, 0) * v * debug->curvature();
  // enable_mpc_feedforward_compensation_默认为false
  // 那么unconstrained_control的计算变得没有意义
  if (enable_mpc_feedforward_compensation_) {
    unconstrained_control_diff =
        Wheel2SteerPct(control[0](0, 0) - unconstrained_control);
    if (fabs(unconstrained_control_diff) <= unconstrained_control_diff_limit_) {
      // control_gain[0]的第三行，相当于k3，但是不同于k3
      // k3 * e2_ss - a_g * v / R 
      steer_angle_ff_compensation =
          Wheel2SteerPct(debug->curvature() *
                         (control_gain[0](0, 2) *
                              (lr_ - lf_ / cr_ * mass_ * v * v / wheelbase_) -
                          addition_gain[0](0, 0) * v));
    } else {
      control_gain_truncation_ratio = control[0](0, 0) / unconstrained_control;
      steer_angle_ff_compensation =
          Wheel2SteerPct(debug->curvature() *
                         (control_gain[0](0, 2) *
                              (lr_ - lf_ / cr_ * mass_ * v * v / wheelbase_) -
                          addition_gain[0](0, 0) * v) *
                         control_gain_truncation_ratio);
    }
  } else {
    steer_angle_ff_compensation = 0.0;
  }

  double mpc_end_timestamp = Clock::NowInSeconds();

  // mpc计算耗时多少毫秒
  ADEBUG << "MPC core algorithm: calculation time is: "
         << (mpc_end_timestamp - mpc_start_timestamp) * 1000 << " ms.";

  // TODO(QiL): evaluate whether need to add spline smoothing after the result
  double steer_angle = steer_angle_feedback +
                       steer_angle_feedforwardterm_updated_ +
                       steer_angle_ff_compensation;

  // FLAGS_set_steer_limit默认为false
  if (FLAGS_set_steer_limit) {
    // 最大横向加速度，阿克曼转向公式，求出最大转角
    // 联系前后文，是否出现了约束多次施加？
    const double steer_limit = std::atan(max_lat_acc_ * wheelbase_ /
                                         (vehicle_state->linear_velocity() *
                                          vehicle_state->linear_velocity())) *
                               steer_ratio_ * 180 / M_PI /
                               steer_single_direction_max_degree_ * 100;

    // Clamp the steer angle with steer limitations at current speed
    double steer_angle_limited =
        common::math::Clamp(steer_angle, -steer_limit, steer_limit);
    steer_angle_limited = digital_filter_.Filter(steer_angle_limited);
    steer_angle = steer_angle_limited;
    debug->set_steer_angle_limited(steer_angle_limited);
  }
  // 滤波器尚未理解透彻，需要进一步深究？
  steer_angle = digital_filter_.Filter(steer_angle);
  // Clamp the steer angle to -100.0 to 100.0
  // 输出方向盘转角百分比的限幅
  steer_angle = common::math::Clamp(steer_angle, -100.0, 100.0);
  cmd->set_steering_target(steer_angle);

  debug->set_acceleration_cmd_closeloop(acc_feedback);

  // 反馈加速度（加速度补偿） 加上 规划下发的期望加速度，得到当前要发下的加速度值
  double acceleration_cmd = acc_feedback + debug->acceleration_reference();
  // TODO(QiL): add pitch angle feed forward to accommodate for 3D control

  // 输出加速度限幅
  if ((planning_published_trajectory->trajectory_type() ==
       apollo::planning::ADCTrajectory::NORMAL) &&
      (std::fabs(debug->acceleration_reference()) <=
           max_acceleration_when_stopped_ &&
       std::fabs(debug->speed_reference()) <= max_abs_speed_when_stopped_)) {
    acceleration_cmd =
        (chassis->gear_location() == canbus::Chassis::GEAR_REVERSE)
            ? std::max(acceleration_cmd, -standstill_acceleration_)
            : std::min(acceleration_cmd, standstill_acceleration_);
    ADEBUG << "Stop location reached";
    debug->set_is_full_stop(true);
  }
  // TODO(Yu): study the necessity of path_remain and add it to MPC if needed

  debug->set_acceleration_cmd(acceleration_cmd);

  double calibration_value = 0.0;
  // FLAGS_use_preview_speed_for_table默认为false
  // 用计算得到的加速度值，和规划下发的参考速度，来查出当前的油门或刹车命令值
  if (FLAGS_use_preview_speed_for_table) {
    calibration_value = control_interpolation_->Interpolate(
        std::make_pair(debug->speed_reference(), acceleration_cmd));
  } else {
    calibration_value = control_interpolation_->Interpolate(std::make_pair(
        injector_->vehicle_state()->linear_velocity(), acceleration_cmd));
  }

  debug->set_calibration_value(calibration_value);

  double throttle_cmd = 0.0;
  double brake_cmd = 0.0;
  if (calibration_value >= 0) {
    throttle_cmd = std::max(calibration_value, throttle_lowerbound_);
    brake_cmd = 0.0;
  } else {
    throttle_cmd = 0.0;
    brake_cmd = std::max(-calibration_value, brake_lowerbound_);
  }

  // lincoln, steer_angle_rate: 6.98rad/s, 400deg/s 
  cmd->set_steering_rate(FLAGS_steer_angle_rate);
  // if the car is driven by acceleration, disgard the cmd->throttle and brake
  cmd->set_throttle(throttle_cmd);
  cmd->set_brake(brake_cmd);
  cmd->set_acceleration(acceleration_cmd);

  debug->set_heading(vehicle_state->heading());
  // 方向盘实际所在位置
  debug->set_steering_position(chassis->steering_percentage());
  debug->set_steer_angle(steer_angle);
  debug->set_steer_angle_feedforward(steer_angle_feedforwardterm_updated_);
  debug->set_steer_angle_feedforward_compensation(steer_angle_ff_compensation);
  debug->set_steer_unconstrained_control_diff(unconstrained_control_diff);
  debug->set_steer_angle_feedback(steer_angle_feedback);
  // debug->set_steering_position(chassis->steering_percentage());

  // 换挡条件是否太宽松了？
  if (std::fabs(vehicle_state->linear_velocity()) <=
          vehicle_param_.max_abs_speed_when_stopped() ||
      chassis->gear_location() == planning_published_trajectory->gear() ||
      chassis->gear_location() == canbus::Chassis::GEAR_NEUTRAL) {
    cmd->set_gear_location(planning_published_trajectory->gear());
  } else {
    cmd->set_gear_location(chassis->gear_location());
  }

  ProcessLogs(debug, chassis);
  return Status::OK();
}

Status MPCController::Reset() {
  previous_heading_error_ = 0.0;
  previous_lateral_error_ = 0.0;
  return Status::OK();
}

void MPCController::LoadControlCalibrationTable(
    const MPCControllerConf &mpc_controller_conf) {
  const auto &control_table = mpc_controller_conf.calibration_table();
  ADEBUG << "Control calibration table loaded";
  ADEBUG << "Control calibration table size is "
         << control_table.calibration_size();
  Interpolation2D::DataType xyz;
  // 创建表
  for (const auto &calibration : control_table.calibration()) {
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.acceleration(),
                                  calibration.command()));
  }
  control_interpolation_.reset(new Interpolation2D);
  ACHECK(control_interpolation_->Init(xyz))
      << "Fail to load control calibration table";
}

void MPCController::UpdateState(SimpleMPCDebug *debug) {
  // 以后轴中心，后轴到质心距离，航向，计算质心的位置，在全局坐标系下计算
  const auto &com = injector_->vehicle_state()->ComputeCOMPosition(lr_);
  ComputeLateralErrors(com.x(), com.y(), injector_->vehicle_state()->heading(),
                       injector_->vehicle_state()->linear_velocity(),
                       injector_->vehicle_state()->angular_velocity(),
                       injector_->vehicle_state()->linear_acceleration(),
                       trajectory_analyzer_, debug);

  // State matrix update;
  // s-l坐标系下的横向误差和航向误差以及他们的变化率
  matrix_state_(0, 0) = debug->lateral_error();
  matrix_state_(1, 0) = debug->lateral_error_rate();
  matrix_state_(2, 0) = debug->heading_error();
  matrix_state_(3, 0) = debug->heading_error_rate();
  matrix_state_(4, 0) = debug->station_error();
  matrix_state_(5, 0) = debug->speed_error();
}

void MPCController::UpdateMatrix(SimpleMPCDebug *debug) {
  const double v = std::max(injector_->vehicle_state()->linear_velocity(),
                            minimum_speed_protection_);
  matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
  matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
  matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
  matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;

  // （A的列数个行，A的列数个列）的单位矩阵
  Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
  // 双线性变换的离散化
  matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
               (matrix_i + ts_ * 0.5 * matrix_a_);


  matrix_c_(1, 0) = (lr_ * cr_ - lf_ * cf_) / mass_ / v - v;
  matrix_c_(3, 0) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_ / v;
  // C * psi_dot 应该是在 X_dot的方程里，因为C的第5个和第6个元素都是零，psi_dot是一个标量，
  // 所以 C * psi_dot 相当于只加在横向误差变化率和航向误差变化率上
  // 这个离散化或许有错？
  matrix_cd_ = matrix_c_ * debug->ref_heading_rate() * ts_;
}

void MPCController::FeedforwardUpdate(SimpleMPCDebug *debug) {
  // 这个方程不需要给Cf，Cr乘以2，其余的都需要
  const double v = injector_->vehicle_state()->linear_velocity();
  const double kv =
      lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;
  //TODO(yxb): 这个公司缺少了第三项
  steer_angle_feedforwardterm_ = Wheel2SteerPct(
      wheelbase_ * debug->curvature() + kv * v * v * debug->curvature());
}

void MPCController::ComputeLateralErrors(
    const double x, const double y, const double theta, const double linear_v,
    const double angular_v, const double linear_a,
    const TrajectoryAnalyzer &trajectory_analyzer, SimpleMPCDebug *debug) {
  // 与QueryMatchedPathPoint不同，这里寻找距离上距离当前位置最近的轨迹上的实际点
  const auto matched_point =
      trajectory_analyzer.QueryNearestPointByPosition(x, y);

  const double dx = x - matched_point.path_point().x();
  const double dy = y - matched_point.path_point().y();

  const double cos_matched_theta = std::cos(matched_point.path_point().theta());
  const double sin_matched_theta = std::sin(matched_point.path_point().theta());
  // d_error = cos_matched_theta * dy - sin_matched_theta * dx;
  // s-l坐标系下的当前位置与“距离最近点”的横向误差
  debug->set_lateral_error(cos_matched_theta * dy - sin_matched_theta * dx);

  // matched_theta = matched_point.path_point().theta();
  // 全局坐标系下的航向角
  debug->set_ref_heading(matched_point.path_point().theta());
  // 全局坐标系下的航向角误差
  const double delta_theta =
      common::math::NormalizeAngle(theta - debug->ref_heading());
  debug->set_heading_error(delta_theta);

  const double sin_delta_theta = std::sin(delta_theta);
  // d_error_dot = chassis_v * sin_delta_theta;
  // s-l坐标系下s方向的速度
  double lateral_error_dot = linear_v * sin_delta_theta;
  // s-l坐标系下s方向的加速度
  double lateral_error_dot_dot = linear_a * sin_delta_theta;
  // FLAGS_reverse_heading_control定义为假，定义后没有再赋值
  if (FLAGS_reverse_heading_control) {
    if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
      lateral_error_dot = -lateral_error_dot;
      lateral_error_dot_dot = -lateral_error_dot_dot;
    }
  }

  debug->set_lateral_error_rate(lateral_error_dot);
  debug->set_lateral_acceleration(lateral_error_dot_dot);
  debug->set_lateral_jerk(
      (debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_);
  previous_lateral_acceleration_ = debug->lateral_acceleration();

  // matched_kappa = matched_point.path_point().kappa();
  debug->set_curvature(matched_point.path_point().kappa());
  // theta_error = delta_theta;
  debug->set_heading_error(delta_theta);
  // theta_error_dot = angular_v - matched_point.path_point().kappa() *
  // matched_point.v();
  // 全局坐标系下，车辆的横摆角速度
  debug->set_heading_rate(angular_v);
  // 车速除以转向半径得到角速度
  debug->set_ref_heading_rate(debug->curvature() * matched_point.v());
  // 航向角误差的变化率，相当于，航向角误差的速度
  debug->set_heading_error_rate(debug->heading_rate() -
                                debug->ref_heading_rate());

  // 当前位置航向角的二阶导，相当于，航向角的角加速度
  debug->set_heading_acceleration(
      (debug->heading_rate() - previous_heading_rate_) / ts_);
  // “距离最近点”的航向角的二阶导，相当于，“距离最近点”的航向角的角加速度
  debug->set_ref_heading_acceleration(
      (debug->ref_heading_rate() - previous_ref_heading_rate_) / ts_);
  // 航向角误差的角加速度
  debug->set_heading_error_acceleration(debug->heading_acceleration() -
                                        debug->ref_heading_acceleration());
  previous_heading_rate_ = debug->heading_rate();
  previous_ref_heading_rate_ = debug->ref_heading_rate();

  // 当前位置的航向角的三阶导
  debug->set_heading_jerk(
      (debug->heading_acceleration() - previous_heading_acceleration_) / ts_);
  // “距离最近点”的航向角的三阶导
  debug->set_ref_heading_jerk(
      (debug->ref_heading_acceleration() - previous_ref_heading_acceleration_) /
      ts_);
  // 航向角三阶导的误差，即“航向角误差的jerk”
  debug->set_heading_error_jerk(debug->heading_jerk() -
                                debug->ref_heading_jerk());
  previous_heading_acceleration_ = debug->heading_acceleration();
  previous_ref_heading_acceleration_ = debug->ref_heading_acceleration();
}

// MPC控制器计算纵向station误差，与lon_controller相同
// 只计算单点误差
void MPCController::ComputeLongitudinalErrors(
    const TrajectoryAnalyzer *trajectory_analyzer, SimpleMPCDebug *debug) {
  // the decomposed vehicle motion onto Frenet frame
  // s: longitudinal accumulated distance along reference trajectory
  // s_dot: longitudinal velocity along reference trajectory
  // d: lateral distance w.r.t. reference trajectory
  // d_dot: lateral distance change rate, i.e. dd/dt
  double s_matched = 0.0;
  double s_dot_matched = 0.0;
  double d_matched = 0.0;
  double d_dot_matched = 0.0;

  // 与lon_controller同，寻找距离当前位置距离最近的轨迹上的“虚拟点”
  const auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
      injector_->vehicle_state()->x(), injector_->vehicle_state()->y());

  // 与lon_controller同
  trajectory_analyzer->ToTrajectoryFrame(
      injector_->vehicle_state()->x(), injector_->vehicle_state()->y(),
      injector_->vehicle_state()->heading(),
      injector_->vehicle_state()->linear_velocity(), matched_point, &s_matched,
      &s_dot_matched, &d_matched, &d_dot_matched);

  const double current_control_time = Clock::NowInSeconds();

  // 与lon_controller同，寻找时间上与当前时间最近的轨迹上的点
  TrajectoryPoint reference_point =
      trajectory_analyzer->QueryNearestPointByAbsoluteTime(
          current_control_time);

  ADEBUG << "matched point:" << matched_point.DebugString();
  ADEBUG << "reference point:" << reference_point.DebugString();

  const double linear_v = injector_->vehicle_state()->linear_velocity();
  const double linear_a = injector_->vehicle_state()->linear_acceleration();
  // 当前车辆航向角与matched_point航向角的差
  double heading_error = common::math::NormalizeAngle(
      injector_->vehicle_state()->heading() - matched_point.theta());
  // "虚拟轨迹点"切线方向速度
  // 规划发送给控制的点带有的变量，不都在同一个坐标系下
  // 比如x，y，z，theta在全局坐标系下,而v，a在s-l坐标系下且都是s方向
  double lon_speed = linear_v * std::cos(heading_error);
  double lon_acceleration = linear_a * std::cos(heading_error);
  double one_minus_kappa_lat_error = 1 - reference_point.path_point().kappa() *
                                             linear_v * std::sin(heading_error);

  debug->set_station_reference(reference_point.path_point().s());
  debug->set_station_feedback(s_matched);
  debug->set_station_error(reference_point.path_point().s() - s_matched);
  debug->set_speed_reference(reference_point.v());
  debug->set_speed_feedback(lon_speed);
  debug->set_speed_error(reference_po int.v() - s_dot_matched);
  debug->set_acceleration_reference(reference_point.a());
  debug->set_acceleration_feedback(lon_acceleration);
  debug->set_acceleration_error(reference_point.a() -
                                lon_acceleration / one_minus_kappa_lat_error);
  double jerk_reference =
      (debug->acceleration_reference() - previous_acceleration_reference_) /
      ts_;
  double lon_jerk =
      (debug->acceleration_feedback() - previous_acceleration_) / ts_;
  debug->set_jerk_reference(jerk_reference);
  debug->set_jerk_feedback(lon_jerk);
  debug->set_jerk_error(jerk_reference - lon_jerk / one_minus_kappa_lat_error);
  previous_acceleration_reference_ = debug->acceleration_reference();
  previous_acceleration_ = debug->acceleration_feedback();
}

}  // namespace control
}  // namespace apollo
