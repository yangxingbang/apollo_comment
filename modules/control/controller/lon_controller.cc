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
#include "modules/control/controller/lon_controller.h"

#include <algorithm>
#include <utility>

#include "absl/strings/str_cat.h"
#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/control/common/control_gflags.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace control {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleStateProvider;
using apollo::cyber::Time;

constexpr double GRA_ACC = 9.8;

LonController::LonController()
    : name_(ControlConf_ControllerType_Name(ControlConf::LON_CONTROLLER)) {
  // 激活它能得到一个csv文件吗？这个文件会在control文件夹内部吗？
  if (FLAGS_enable_csv_debug) {
    // debug内容包括：时间，名字，各种不同的时间，
    time_t rawtime;
    char name_buffer[80];
    std::time(&rawtime);
    std::tm time_tm;
    localtime_r(&rawtime, &time_tm);
    strftime(name_buffer, 80, "/tmp/speed_log__%F_%H%M%S.csv", &time_tm);
    // 采用写的方式打开速度log文件
    // 为什么单单打开速度的log文件？
    speed_log_file_ = fopen(name_buffer, "w");
    if (speed_log_file_ == nullptr) {
      AERROR << "Fail to open file:" << name_buffer;
      FLAGS_enable_csv_debug = false;
    }
    if (speed_log_file_ != nullptr) {
    // 参考站位，站位误差，站位误差限值，预瞄站位误差
    // 参考速度，速度误差，误差速度限值，预瞄参考速度，预瞄速度误差
    // 预瞄参考加速度，加速度命令闭环，加速度命令，加速度查表值
    // 速度查表值，标定值（哪个标定值？），油门命令，制动命令，是否全制动刹停
      fprintf(speed_log_file_,
              "station_reference,"
              "station_error,"
              "station_error_limited,"
              "preview_station_error,"
              "speed_reference,"
              "speed_error,"
              "speed_error_limited,"
              "preview_speed_reference,"
              "preview_speed_error,"
              "preview_acceleration_reference,"
              "acceleration_cmd_closeloop,"
              "acceleration_cmd,"
              "acceleration_lookup,"
              "speed_lookup,"
              "calibration_value,"
              "throttle_cmd,"
              "brake_cmd,"
              "is_full_stop,"
              "\r\n");

      // fflush是一个在C语言标准输入输出库中的函数
      // 功能是冲洗流中的信息，该函数通常用于处理磁盘文件
      fflush(speed_log_file_);
    }
    AINFO << name_ << " used.";
  }
}

void LonController::CloseLogFile() {
  if (FLAGS_enable_csv_debug) {
    if (speed_log_file_ != nullptr) {
      fclose(speed_log_file_);
      speed_log_file_ = nullptr;
    }
  }
}
void LonController::Stop() { CloseLogFile(); }

LonController::~LonController() { CloseLogFile(); }

Status LonController::Init(std::shared_ptr<DependencyInjector> injector,
                           const ControlConf *control_conf) {
  control_conf_ = control_conf;
  if (control_conf_ == nullptr) {
    controller_initialized_ = false;
    AERROR << "get_longitudinal_param() nullptr";
    // 初始化错误消息，返回
    return Status(ErrorCode::CONTROL_INIT_ERROR,
                  "Failed to load LonController conf");
  }
  injector_ = injector;
  const LonControllerConf &lon_controller_conf =
      control_conf_->lon_controller_conf();
  double ts = lon_controller_conf.ts();
  // 是否激活leadlag，reverse表示倒挡
  bool enable_leadlag =
      lon_controller_conf.enable_reverse_leadlag_compensation();

  station_pid_controller_.Init(lon_controller_conf.station_pid_conf());
  speed_pid_controller_.Init(lon_controller_conf.low_speed_pid_conf());

  if (enable_leadlag) {
    station_leadlag_controller_.Init(
        lon_controller_conf.reverse_station_leadlag_conf(), ts);
    speed_leadlag_controller_.Init(
        lon_controller_conf.reverse_speed_leadlag_conf(), ts);
  }

  vehicle_param_.CopyFrom(
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param());
  // 低通滤波器
  SetDigitalFilterPitchAngle(lon_controller_conf);

  LoadControlCalibrationTable(lon_controller_conf);
  controller_initialized_ = true;

  return Status::OK();
}

void LonController::SetDigitalFilterPitchAngle(
    const LonControllerConf &lon_controller_conf) {
  double cutoff_freq =
      lon_controller_conf.pitch_angle_filter_conf().cutoff_freq();
  double ts = lon_controller_conf.ts();
  SetDigitalFilter(ts, cutoff_freq, &digital_filter_pitch_angle_);
}

void LonController::LoadControlCalibrationTable(
    const LonControllerConf &lon_controller_conf) {
  const auto &control_table = lon_controller_conf.calibration_table();
  AINFO << "Control calibration table loaded";
  AINFO << "Control calibration table size is "
        << control_table.calibration_size();
  // 数据2维插值，这里只是声明，还没有开始插值操作
  Interpolation2D::DataType xyz;
  for (const auto &calibration : control_table.calibration()) {
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.acceleration(),
                                  calibration.command()));
  }
  control_interpolation_.reset(new Interpolation2D);
  ACHECK(control_interpolation_->Init(xyz))
      << "Fail to load control calibration table";
}

Status LonController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    control::ControlCommand *cmd) {
  localization_ = localization;
  chassis_ = chassis;

  trajectory_message_ = planning_published_trajectory;
  // 标定表的数据一定要插值，否则报错
  if (!control_interpolation_) {
    AERROR << "Fail to initialize calibration table.";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "Fail to initialize calibration table.");
  }

  // （包含N个点的）轨迹消息会形成一个队列，也就是说这个队列中有多条轨迹消息
  // 队列中的每一个轨迹消息都有编号，当前取到的消息也会有一个编号
  // 如果当前使用的轨迹消息的队列编号和规划发送来的轨迹消息的队列编号不一致
  // 把当前轨迹消息清空
  if (trajectory_analyzer_ == nullptr ||
      trajectory_analyzer_->seq_num() !=
          trajectory_message_->header().sequence_num()) {
    trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
  }
  const LonControllerConf &lon_controller_conf =
      control_conf_->lon_controller_conf();
  
  // 每次进入控制命令计算函数，都把控制数据清零
  auto debug = cmd->mutable_debug()->mutable_simple_lon_debug();
  debug->Clear();

  double brake_cmd = 0.0;
  double throttle_cmd = 0.0;
  double ts = lon_controller_conf.ts();
  // 时间预瞄：20 * 0.01 = 0.2s
  double preview_time = lon_controller_conf.preview_window() * ts;
  // 纵向控制中enable_reverse_leadlag_compensation默认为false
  bool enable_leadlag =
      lon_controller_conf.enable_reverse_leadlag_compensation();

  if (preview_time < 0.0) {
    const auto error_msg =
        absl::StrCat("Preview time set as: ", preview_time, " less than 0");
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }
  // 计算match，reference，preview点和相互之间的误差
  ComputeLongitudinalErrors(trajectory_analyzer_.get(), preview_time, ts,
                            debug);

  double station_error_limit = lon_controller_conf.station_error_limit();
  double station_error_limited = 0.0;
  // 假如有预瞄，就根据预瞄点的位置和速度计算误差
  if (FLAGS_enable_speed_station_preview) {
    station_error_limited =
        common::math::Clamp(debug->preview_station_error(),
                            -station_error_limit, station_error_limit);
  } else {
    station_error_limited = common::math::Clamp(
        debug->station_error(), -station_error_limit, station_error_limit);
  }

  if (trajectory_message_->gear() == canbus::Chassis::GEAR_REVERSE) {
    // 倒挡时只有一种PID参数
    // 0.4, 0.1, 0.0
    station_pid_controller_.SetPID(
        lon_controller_conf.reverse_station_pid_conf());
    // 2.0, 0.3, 0.0
    speed_pid_controller_.SetPID(lon_controller_conf.reverse_speed_pid_conf());
    if (enable_leadlag) {
      // 1.0, 1.0
      station_leadlag_controller_.SetLeadlag(
          lon_controller_conf.reverse_station_leadlag_conf());
      // 1.0, 1.0
      speed_leadlag_controller_.SetLeadlag(
          lon_controller_conf.reverse_speed_leadlag_conf());
    }
  // 前进挡时， 存在速度切换临界值switch_speed = 3.0m/s
  // PID以速度3分段设置参数，3以下为低速
  } else if (injector_->vehicle_state()->linear_velocity() <=
             lon_controller_conf.switch_speed()) {
    // 2.0, 0.3, 0.0
    speed_pid_controller_.SetPID(lon_controller_conf.low_speed_pid_conf());
  } else {
    // 1.0, 0.3, 0.0
    // 假如当前低速和高速PID给定参数是符合逻辑的
    // 那么高速时选用低P值，响应速度降低，减少震荡，降低超调，同时减少初始阶段稳态误差
    speed_pid_controller_.SetPID(lon_controller_conf.high_speed_pid_conf());
  }

  // 执行位置环的PID算法
  double speed_offset =
      station_pid_controller_.Control(station_error_limited, ts);
  if (enable_leadlag) {
    speed_offset = station_leadlag_controller_.Control(speed_offset, ts);
  }

  double speed_controller_input = 0.0;
  double speed_controller_input_limit =
      lon_controller_conf.speed_controller_input_limit();
  double speed_controller_input_limited = 0.0;
  if (FLAGS_enable_speed_station_preview) {
    speed_controller_input = speed_offset + debug->preview_speed_error();
  } else {
    speed_controller_input = speed_offset + debug->speed_error();
  }
  speed_controller_input_limited =
      common::math::Clamp(speed_controller_input, -speed_controller_input_limit,
                          speed_controller_input_limit);

  double acceleration_cmd_closeloop = 0.0;

  // 执行速度环的PID算法
  acceleration_cmd_closeloop =
      speed_pid_controller_.Control(speed_controller_input_limited, ts);
  debug->set_pid_saturation_status(
      speed_pid_controller_.IntegratorSaturationStatus());
  // 超前滞后串联在PID后边
  if (enable_leadlag) {
    acceleration_cmd_closeloop =
        speed_leadlag_controller_.Control(acceleration_cmd_closeloop, ts);
    debug->set_leadlag_saturation_status(
        speed_leadlag_controller_.InnerstateSaturationStatus());
  }

   // 滤波
  double slope_offset_compenstaion = digital_filter_pitch_angle_.Filter(
      GRA_ACC * std::sin(injector_->vehicle_state()->pitch()));

  if (std::isnan(slope_offset_compenstaion)) {
    slope_offset_compenstaion = 0;
  }

  debug->set_slope_offset_compensation(slope_offset_compenstaion);

  double acceleration_cmd =
      acceleration_cmd_closeloop + debug->preview_acceleration_reference() +
      FLAGS_enable_slope_offset * debug->slope_offset_compensation();
  debug->set_is_full_stop(false);
  // 拿到剩余路径长度和停车轨迹点的索引
  GetPathRemain(debug);

  // At near-stop stage, replace the brake control command with the standstill
  // acceleration if the former is even softer than the latter
  // 假如之前的刹车力度太小，剩余距离太小了，将制动命令变更为standstill加速度
  // 并且激活全面刹停标志位
  // 加速度边界0.01， 车速边界0.2，距离边界0.3
  if ((trajectory_message_->trajectory_type() ==
       apollo::planning::ADCTrajectory::NORMAL) &&
      ((std::fabs(debug->preview_acceleration_reference()) <=
            control_conf_->max_acceleration_when_stopped() &&
        std::fabs(debug->preview_speed_reference()) <=
            vehicle_param_.max_abs_speed_when_stopped()) ||
       std::abs(debug->path_remain()) <
           control_conf_->max_path_remain_when_stopped())) {
    acceleration_cmd =
        (chassis->gear_location() == canbus::Chassis::GEAR_REVERSE)
            ? std::max(acceleration_cmd,
                       -lon_controller_conf.standstill_acceleration())
            : std::min(acceleration_cmd,
                       lon_controller_conf.standstill_acceleration());
    ADEBUG << "Stop location reached";
    debug->set_is_full_stop(true);
  }

  // 世界上存在死区比最小可动作值更小的执行器吗？
  double throttle_lowerbound =
      std::max(vehicle_param_.throttle_deadzone(),
               lon_controller_conf.throttle_minimum_action());
  double brake_lowerbound =
      std::max(vehicle_param_.brake_deadzone(),
               lon_controller_conf.brake_minimum_action());
  double calibration_value = 0.0;
  // 定义加速度的查表值
  double acceleration_lookup =
      (chassis->gear_location() == canbus::Chassis::GEAR_REVERSE)
          ? -acceleration_cmd
          : acceleration_cmd;

  // 假如使用了预瞄点的速度来查表，那么按预瞄点的速度查表，否则按当前速度查表
  if (FLAGS_use_preview_speed_for_table) {
    calibration_value = control_interpolation_->Interpolate(
        std::make_pair(debug->preview_speed_reference(), acceleration_lookup));
  } else {
    calibration_value = control_interpolation_->Interpolate(
        std::make_pair(chassis_->speed_mps(), acceleration_lookup));
  }

  // 没有油门制动一起踩
  if (acceleration_lookup >= 0) {
    if (calibration_value >= 0) {
      // 加速度值为正，标定值也为正，那么取大作为油门开度
      throttle_cmd = std::max(calibration_value, throttle_lowerbound);
    } else {
      // 这里加速度查表值为正，标定值为负
      // 
      throttle_cmd = throttle_lowerbound;
    }
    brake_cmd = 0.0;
  } else {
    throttle_cmd = 0.0;
    if (calibration_value >= 0) {
      brake_cmd = brake_lowerbound;
    } else {
      brake_cmd = std::max(-calibration_value, brake_lowerbound);
    }
  }

  debug->set_station_error_limited(station_error_limited);
  debug->set_speed_offset(speed_offset);
  debug->set_speed_controller_input_limited(speed_controller_input_limited);
  debug->set_acceleration_cmd(acceleration_cmd);
  debug->set_throttle_cmd(throttle_cmd);
  debug->set_brake_cmd(brake_cmd);
  debug->set_acceleration_lookup(acceleration_lookup);
  debug->set_speed_lookup(chassis_->speed_mps());
  debug->set_calibration_value(calibration_value);
  debug->set_acceleration_cmd_closeloop(acceleration_cmd_closeloop);

  if (FLAGS_enable_csv_debug && speed_log_file_ != nullptr) {
    fprintf(speed_log_file_,
            "%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f,"
            "%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %d,\r\n",
            debug->station_reference(), debug->station_error(),
            station_error_limited, debug->preview_station_error(),
            debug->speed_reference(), debug->speed_error(),
            speed_controller_input_limited, debug->preview_speed_reference(),
            debug->preview_speed_error(),
            debug->preview_acceleration_reference(), acceleration_cmd_closeloop,
            acceleration_cmd, debug->acceleration_lookup(),
            debug->speed_lookup(), calibration_value, throttle_cmd, brake_cmd,
            debug->is_full_stop());
  }

  // if the car is driven by acceleration, disgard the cmd->throttle and brake
  cmd->set_throttle(throttle_cmd);
  cmd->set_brake(brake_cmd);
  cmd->set_acceleration(acceleration_cmd);
  // 车速低于0.2，或者当前挡位是轨迹消息挡位，或者空挡,都设定挡位
  // 这样的话，车辆每10ms就会接到一个换挡信号
  if (std::fabs(injector_->vehicle_state()->linear_velocity()) <=
          vehicle_param_.max_abs_speed_when_stopped() ||
      chassis->gear_location() == trajectory_message_->gear() ||
      chassis->gear_location() == canbus::Chassis::GEAR_NEUTRAL) {
    cmd->set_gear_location(trajectory_message_->gear());
  } else {
    cmd->set_gear_location(chassis->gear_location());
  }

  return Status::OK();
}

Status LonController::Reset() {
  speed_pid_controller_.Reset();
  station_pid_controller_.Reset();
  return Status::OK();
}

std::string LonController::Name() const { return name_; }

void LonController::ComputeLongitudinalErrors(
    const TrajectoryAnalyzer *trajectory_analyzer, const double preview_time,
    const double ts, SimpleLongitudinalDebug *debug) {
  // 车辆运动基于Frenet坐标系
  // the decomposed vehicle motion onto Frenet frame
  // s: longitudinal accumulated distance along reference trajectory
  // s_dot: longitudinal velocity along reference trajectory
  // d: lateral distance w.r.t. reference trajectory
  // d_dot: lateral distance change rate, i.e. dd/dt
  double s_matched = 0.0;
  double s_dot_matched = 0.0;
  double d_matched = 0.0;
  double d_dot_matched = 0.0;

  auto vehicle_state = injector_->vehicle_state();
  // x,y在全局坐标系下
  // 取当前车辆位置的路径上最近点，该点插值得到，不是带索引的轨迹点，我称它为“虚拟点”
  auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
      vehicle_state->x(), vehicle_state->y());
  // 全局坐标系系转换到Frenet坐标系
  // s_matched是在frenet坐标系中求出matched_point与车辆当前位置的s误差，
  // 然后得到的车辆当前位置在S-L中的s值
  trajectory_analyzer->ToTrajectoryFrame(
      vehicle_state->x(), vehicle_state->y(), vehicle_state->heading(),
      vehicle_state->linear_velocity(), matched_point, &s_matched,
      &s_dot_matched, &d_matched, &d_dot_matched);

  // 当前时间加上预瞄时间，得到"预瞄控制时间"
  // chrono是一个time library, 源于boost，现在已经是C++标准
  double current_control_time = Time::Now().ToSecond();
  double preview_control_time = current_control_time + preview_time;

  // reference点是和时间相关的，而上边的match点是和距离相关的
  // 当前位置时间戳与轨迹上点时间戳 最接近的 那个轨迹点
  TrajectoryPoint reference_point =
      trajectory_analyzer->QueryNearestPointByAbsoluteTime(
          current_control_time);
  // “预瞄控制时间”的轨迹最近点
  TrajectoryPoint preview_point =
      trajectory_analyzer->QueryNearestPointByAbsoluteTime(
          preview_control_time);

  debug->mutable_current_matched_point()->mutable_path_point()->set_x(
      matched_point.x());
  debug->mutable_current_matched_point()->mutable_path_point()->set_y(
      matched_point.y());
  debug->mutable_current_reference_point()->mutable_path_point()->set_x(
      reference_point.path_point().x());
  debug->mutable_current_reference_point()->mutable_path_point()->set_y(
      reference_point.path_point().y());
  debug->mutable_preview_reference_point()->mutable_path_point()->set_x(
      preview_point.path_point().x());
  debug->mutable_preview_reference_point()->mutable_path_point()->set_y(
      preview_point.path_point().y());

  ADEBUG << "matched point:" << matched_point.DebugString();
  ADEBUG << "reference point:" << reference_point.DebugString();
  ADEBUG << "preview point:" << preview_point.DebugString();

  // 归一化到0到2*pi之间
  // 注意这三个量和ToTrajectoryFrame中的计算方式不同，所以它们在全部坐标系
  double heading_error = common::math::NormalizeAngle(vehicle_state->heading() -
                                                      matched_point.theta());
  double lon_speed = vehicle_state->linear_velocity() * std::cos(heading_error);
  double lon_acceleration =
      vehicle_state->linear_acceleration() * std::cos(heading_error);
  double one_minus_kappa_lat_error = 1 - reference_point.path_point().kappa() *
                                             vehicle_state->linear_velocity() *
                                             std::sin(heading_error);

  // 距离当前位置时间上最近的轨迹点
  debug->set_station_reference(reference_point.path_point().s());
  // s_matched是在frenet坐标系中求出matched_point与车辆当前位置的s误差，
  // 然后得到的车辆当前位置在S-L中的s值
  debug->set_current_station(s_matched);
  // S-L坐标系下的纵向位移差delta_s
  debug->set_station_error(reference_point.path_point().s() - s_matched);
  debug->set_speed_reference(reference_point.v());
  debug->set_current_speed(lon_speed);
  // 时间最近点和距离最近点的速度差
  debug->set_speed_error(reference_point.v() - s_dot_matched);
  debug->set_acceleration_reference(reference_point.a());
  debug->set_current_acceleration(lon_acceleration);
  // 公式？
  debug->set_acceleration_error(reference_point.a() -
                                lon_acceleration / one_minus_kappa_lat_error);
  double jerk_reference =
      (debug->acceleration_reference() - previous_acceleration_reference_) / ts;
  double lon_jerk =
      (debug->current_acceleration() - previous_acceleration_) / ts;
  debug->set_jerk_reference(jerk_reference);
  debug->set_current_jerk(lon_jerk);
  // 公式？
  debug->set_jerk_error(jerk_reference - lon_jerk / one_minus_kappa_lat_error);
  previous_acceleration_reference_ = debug->acceleration_reference();
  previous_acceleration_ = debug->current_acceleration();

  debug->set_preview_station_error(preview_point.path_point().s() - s_matched);
  debug->set_preview_speed_error(preview_point.v() - s_dot_matched);
  debug->set_preview_speed_reference(preview_point.v());
  debug->set_preview_acceleration_reference(preview_point.a());
}

void LonController::SetDigitalFilter(double ts, double cutoff_freq,
                                     common::DigitalFilter *digital_filter) {
  std::vector<double> denominators;
  std::vector<double> numerators;
  // 计算低通滤波器的系数
  common::LpfCoefficients(ts, cutoff_freq, &denominators, &numerators);
  // 传递低通滤波器的系数
  digital_filter->set_coefficients(denominators, numerators);
}

// TODO(all): Refactor and simplify
void LonController::GetPathRemain(SimpleLongitudinalDebug *debug) {
  int stop_index = 0;
  static constexpr double kSpeedThreshold = 1e-3;
  static constexpr double kForwardAccThreshold = -1e-2;
  static constexpr double kBackwardAccThreshold = 1e-1;
  static constexpr double kParkingSpeed = 0.1;

  if (trajectory_message_->gear() == canbus::Chassis::GEAR_DRIVE) {
    while (stop_index < trajectory_message_->trajectory_point_size()) {
      auto &current_trajectory_point =
          trajectory_message_->trajectory_point(stop_index);
      if (fabs(current_trajectory_point.v()) < kSpeedThreshold &&
          current_trajectory_point.a() > kForwardAccThreshold &&
          current_trajectory_point.a() < 0.0) {
        break;
      }
      ++stop_index;
    }
  } else {
    while (stop_index < trajectory_message_->trajectory_point_size()) {
      auto &current_trajectory_point =
          trajectory_message_->trajectory_point(stop_index);
      if (current_trajectory_point.v() < kSpeedThreshold &&
          current_trajectory_point.a() < kBackwardAccThreshold &&
          current_trajectory_point.a() > 0.0) {
        break;
      }
      ++stop_index;
    }
  }
  if (stop_index == trajectory_message_->trajectory_point_size()) {
    --stop_index;
    if (fabs(trajectory_message_->trajectory_point(stop_index).v()) <
        kParkingSpeed) {
      ADEBUG << "the last point is selected as parking point";
    } else {
      ADEBUG << "the last point found in path and speed > speed_deadzone";
    }
  }
  debug->set_path_remain(
      trajectory_message_->trajectory_point(stop_index).path_point().s() -
      debug->current_station());
}

}  // namespace control
}  // namespace apollo
