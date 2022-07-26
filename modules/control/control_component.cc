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
#include "modules/control/control_component.h"

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/latency_recorder/latency_recorder.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::cyber::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

// MonitorMessageItem包括20个消息源（各个功能模块），具体的msg（消息），各个级别的log打印信息
// 全部发送到monitor,参见monitor_log.proto
// 这里初始化一个消息源
ControlComponent::ControlComponent()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

bool ControlComponent::Init() {
  injector_ = std::make_shared<DependencyInjector>();
  init_time_ = Clock::Now();

  AINFO << "Control init, starting ...";

  // /apollo/modules/control/conf/control_conf.pb.txt
  ACHECK(
      cyber::common::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_))
      << "Unable to load control conf file: " + FLAGS_control_conf_file;

  AINFO << "Conf file: " << FLAGS_control_conf_file << " is loaded.";
  // config文件目录在哪里，在dag？还是与control_conf在同一目录？
  // 可以在terminal打印出来，cyber/setup.bash中GLOG_minloglev = 0
  AINFO << "Conf file: " << ConfigFilePath() << " is loaded.";

  // initial controller agent when not using control submodules
  // control子模型分为pre，controller，post三个子模型，每一个都是单独的component
  // 这就相当于把控制拆分成三个部分, 而且都不是时间触发的timer_component
  // 那这些模块怎么启动，相互间怎么调用？
  // 这么做有什么好处吗？方便进程调度来数据处理吗？
  // 实际上代码中并没有给出使用子模型的方法实现
  ADEBUG << "FLAGS_use_control_submodules: " << FLAGS_use_control_submodules;
  if (!FLAGS_use_control_submodules &&
      !controller_agent_.Init(injector_, &control_conf_).ok()) {
    // set controller
    ADEBUG << "original control";
    monitor_logger_buffer_.ERROR("Control init controller failed! Stopping...");
    return false;
  }
  // ReaderConfig是个结构体，包含reader的通道名，挂起队列等
  // 利用reader_config去创建reader
  cyber::ReaderConfig chassis_reader_config;
  chassis_reader_config.channel_name = FLAGS_chassis_topic;
  chassis_reader_config.pending_queue_size = FLAGS_chassis_pending_queue_size;

  chassis_reader_ =
      node_->CreateReader<Chassis>(chassis_reader_config, nullptr);
  ACHECK(chassis_reader_ != nullptr);

  // planning_reader_config创建的是trajectory_reader 
  cyber::ReaderConfig planning_reader_config;
  planning_reader_config.channel_name = FLAGS_planning_trajectory_topic;
  planning_reader_config.pending_queue_size = FLAGS_planning_pending_queue_size;

  trajectory_reader_ =
      node_->CreateReader<ADCTrajectory>(planning_reader_config, nullptr);
  ACHECK(trajectory_reader_ != nullptr);

  cyber::ReaderConfig localization_reader_config;
  localization_reader_config.channel_name = FLAGS_localization_topic;
  localization_reader_config.pending_queue_size =
      FLAGS_localization_pending_queue_size;

  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      localization_reader_config, nullptr);
  ACHECK(localization_reader_ != nullptr);

  // 从平板上点击dreamview界面的按钮，按钮是前端，前端与后端通过websocket通讯
  // 后端把按钮操作变为实际变量信息，设入到pad_topic中
  // pad_topic中的消息在control component中被reader读到，然后转存到pad_msg_
  // 最后设入到control command，作为后续开启自动控制的状态量
  cyber::ReaderConfig pad_msg_reader_config;
  pad_msg_reader_config.channel_name = FLAGS_pad_topic;
  pad_msg_reader_config.pending_queue_size = FLAGS_pad_msg_pending_queue_size;

  pad_msg_reader_ =
      node_->CreateReader<PadMessage>(pad_msg_reader_config, nullptr);
  ACHECK(pad_msg_reader_ != nullptr);

  // 不使用控制子模型，就创建control_cmd_writer_，否则创建local_view_writer_
  if (!FLAGS_use_control_submodules) {
    control_cmd_writer_ =
        node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);
    ACHECK(control_cmd_writer_ != nullptr);
  } else {
    local_view_writer_ =
        node_->CreateWriter<LocalView>(FLAGS_control_local_view_topic);
    ACHECK(local_view_writer_ != nullptr);
  }

  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // should init_vehicle first, let car enter work status, then use status msg
  // trigger control

  // 从control_conf.pb.txt中读取STOP作为初值，OnPad修改
  AINFO << "Control default driving action is "
        << DrivingAction_Name(control_conf_.action());
  pad_msg_.set_action(control_conf_.action());

  return true;
}

// 检测pad消息中的动作，假如消息中没有action就报错
// 这个存在的action将会覆盖初始化函数中的STOP action
void ControlComponent::OnPad(const std::shared_ptr<PadMessage> &pad) {
  std::lock_guard<std::mutex> lock(mutex_);
  pad_msg_.CopyFrom(*pad);
  ADEBUG << "Received Pad Msg:" << pad_msg_.DebugString();
  AERROR_IF(!pad_msg_.has_action()) << "pad message check failed!";
}

// 以下循环发送的消息都要拿最新的
void ControlComponent::OnChassis(const std::shared_ptr<Chassis> &chassis) {
  ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_chassis_.CopyFrom(*chassis);
}

void ControlComponent::OnPlanning(
    const std::shared_ptr<ADCTrajectory> &trajectory) {
  ADEBUG << "Received chassis data: run trajectory callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_trajectory_.CopyFrom(*trajectory);
}

void ControlComponent::OnLocalization(
    const std::shared_ptr<LocalizationEstimate> &localization) {
  ADEBUG << "Received control data: run localization message callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_localization_.CopyFrom(*localization);
}

// item就是MonitorMessageItem，假如这些消息源的打印log信息的级别是致命的
// 就把estop_置为真，estop_不会发送出去，只在控制内部起作用
void ControlComponent::OnMonitor(
    const common::monitor::MonitorMessage &monitor_message) {
  for (const auto &item : monitor_message.item()) {
    if (item.log_level() == common::monitor::MonitorMessageItem::FATAL) {
      estop_ = true;
      return;
    }
  }
}

Status ControlComponent::ProduceControlCommand(
    ControlCommand *control_command) {
  Status status = CheckInput(&local_view_);
  // check data
  if (!status.ok()) {
    AERROR_EVERY(100) << "Control input data failed: "
                      << status.error_message();
    // 假如控制输入有问题，不允许进入自动驾驶模式
    control_command->mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    control_command->mutable_engage_advice()->set_reason(
        status.error_message());
    estop_ = true;
    estop_reason_ = status.error_message();
  } else {
    Status status_ts = CheckTimestamp(local_view_);
    // 检查控制输入数据的时间戳
    if (!status_ts.ok()) {
      AERROR << "Input messages timeout";
      // estop_ = true;
      status = status_ts;
      // 假如底盘回传的驾驶模式不再是“完全自动驾驶”，控制模块建议“不再继续自动驾驶”
      if (local_view_.chassis().driving_mode() !=
          apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
        control_command->mutable_engage_advice()->set_advice(
            apollo::common::EngageAdvice::DISALLOW_ENGAGE);
        control_command->mutable_engage_advice()->set_reason(
            status.error_message());
      }
    } else {
      // 控制输入数据时间戳无误，就准备好自动驾驶
      control_command->mutable_engage_advice()->set_advice(
          apollo::common::EngageAdvice::READY_TO_ENGAGE);
    }
  }

  // check estop
  // enable_persistent_estop什么意思？持续发送18个循环吗？
  // 还是仅仅只是默认为真的标志位用来做判断的？
  estop_ = control_conf_.enable_persistent_estop()
               ? estop_ || local_view_.trajectory().estop().is_estop()
               : local_view_.trajectory().estop().is_estop();

  if (local_view_.trajectory().estop().is_estop()) {
    estop_ = true;
    estop_reason_ = "estop from planning : ";
    estop_reason_ += local_view_.trajectory().estop().reason();
  }

  // CheckInput虽然校验了轨迹点是否空，但是没有赋estop值 
  if (local_view_.trajectory().trajectory_point().empty()) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    estop_ = true;
    estop_reason_ = "estop for empty planning trajectory, planning headers: " +
                    local_view_.trajectory().header().ShortDebugString();
  }

  // 前进挡的第一个轨迹点一定要是正速度
  if (FLAGS_enable_gear_drive_negative_speed_protection) {
    const double kEpsilon = 0.001;
    auto first_trajectory_point = local_view_.trajectory().trajectory_point(0);
    if (local_view_.chassis().gear_location() == Chassis::GEAR_DRIVE &&
        first_trajectory_point.v() < -1 * kEpsilon) {
      estop_ = true;
      estop_reason_ = "estop for negative speed when gear_drive";
    }
  }

  if (!estop_) {
    // 在这里判定了自动驾驶还是手动驾驶！！！
    if (local_view_.chassis().driving_mode() == Chassis::COMPLETE_MANUAL) {
      controller_agent_.Reset();
      AINFO_EVERY(100) << "Reset Controllers in Manual Mode";
    }

    // 这个debug指针只能复制输入消息，别的类型的消息复制不了
    auto debug = control_command->mutable_debug()->mutable_input_debug();
    debug->mutable_localization_header()->CopyFrom(
        local_view_.localization().header());
    debug->mutable_canbus_header()->CopyFrom(local_view_.chassis().header());
    debug->mutable_trajectory_header()->CopyFrom(
        local_view_.trajectory().header());
    // 假如是重规划,就把重规划后的消息头给最新消息变量
    if (local_view_.trajectory().is_replan()) {
      latest_replan_trajectory_header_ = local_view_.trajectory().header();
    }

    if (latest_replan_trajectory_header_.has_sequence_num()) {
      debug->mutable_latest_replan_trajectory_header()->CopyFrom(
          latest_replan_trajectory_header_);
    }
    // controller agent
    Status status_compute = controller_agent_.ComputeControlCommand(
        &local_view_.localization(), &local_view_.chassis(),
        &local_view_.trajectory(), control_command);

    if (!status_compute.ok()) {
      AERROR << "Control main function failed"
             << " with localization: "
             << local_view_.localization().ShortDebugString()
             << " with chassis: " << local_view_.chassis().ShortDebugString()
             << " with trajectory: "
             << local_view_.trajectory().ShortDebugString()
             << " with cmd: " << control_command->ShortDebugString()
             << " status:" << status_compute.error_message();
      estop_ = true;
      estop_reason_ = status_compute.error_message();
      status = status_compute;
    }
  }
  // if planning set estop, then no control process triggered
  // 如果规划发送的消息中estop为真，控制不会运行，而且要保持复位状态
  if (estop_) {
    AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
    // set Estop command
    control_command->set_speed(0);
    control_command->set_throttle(0);
    // 软刹, 50%制动踏板开度
    control_command->set_brake(control_conf_.soft_estop_brake());
    // 没有控制命令输出的时候，怎么能挂D挡？
    control_command->set_gear_location(Chassis::GEAR_DRIVE);
  }
  // check signal
  
  // vehicle_signal指的是转向灯，远近光灯， 喇叭，双闪灯的结构体
  if (local_view_.trajectory().decision().has_vehicle_signal()) {
    control_command->mutable_signal()->CopyFrom(
        local_view_.trajectory().decision().vehicle_signal());
  }
  return status;
}

bool ControlComponent::Proc() {
  const auto start_time = Clock::Now();
  // 得到 发布的消息队列
  chassis_reader_->Observe();
  // 获取消息队列最前端的元素
  const auto &chassis_msg = chassis_reader_->GetLatestObserved();
  if (chassis_msg == nullptr) {
    AERROR << "Chassis msg is not ready!";
    return false;
  }
  // 把队列最前端的chassis_msg复制到latest_chassis_
  OnChassis(chassis_msg);

  trajectory_reader_->Observe();
  const auto &trajectory_msg = trajectory_reader_->GetLatestObserved();
  if (trajectory_msg == nullptr) {
    AERROR << "planning msg is not ready!";
    return false;
  }
  OnPlanning(trajectory_msg);

  localization_reader_->Observe();
  const auto &localization_msg = localization_reader_->GetLatestObserved();
  if (localization_msg == nullptr) {
    AERROR << "localization msg is not ready!";
    return false;
  }
  OnLocalization(localization_msg);

  pad_msg_reader_->Observe();
  const auto &pad_msg = pad_msg_reader_->GetLatestObserved();
  if (pad_msg != nullptr) {
    OnPad(pad_msg);
  }

  {
    // TODO(SHU): to avoid redundent copy
    // 这里表示的是，不需要那么繁琐的层层叠叠的拷贝方式，
    // 用这块代替以上好几步的拷贝操作？
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.mutable_chassis()->CopyFrom(latest_chassis_);
    local_view_.mutable_trajectory()->CopyFrom(latest_trajectory_);
    local_view_.mutable_localization()->CopyFrom(latest_localization_);
    if (pad_msg != nullptr) {
      local_view_.mutable_pad_msg()->CopyFrom(pad_msg_);
    }
  }

  // use control submodules  
  if (FLAGS_use_control_submodules) {
    local_view_.mutable_header()->set_lidar_timestamp(
        local_view_.trajectory().header().lidar_timestamp());
    local_view_.mutable_header()->set_camera_timestamp(
        local_view_.trajectory().header().camera_timestamp());
    local_view_.mutable_header()->set_radar_timestamp(
        local_view_.trajectory().header().radar_timestamp());
    common::util::FillHeader(FLAGS_control_local_view_topic, &local_view_);

    const auto end_time = Clock::Now();

    // measure latency
    // 延迟
    static apollo::common::LatencyRecorder latency_recorder(
        FLAGS_control_local_view_topic);
    // 这里为什么要加lidar的时间戳？这和控制有什么关系？
    latency_recorder.AppendLatencyRecord(
        local_view_.trajectory().header().lidar_timestamp(), start_time,
        end_time);

    local_view_writer_->Write(local_view_);
    return true;
  }

  if (pad_msg != nullptr) {
    ADEBUG << "pad_msg: " << pad_msg_.ShortDebugString();
    if (pad_msg_.action() == DrivingAction::RESET) {
      AINFO << "Control received RESET action!";
      estop_ = false;
      estop_reason_.clear();
    }
    pad_received_ = true;
  }

  // 单元测试模式默认为假，测试时长默认为-1
  if (control_conf_.is_control_test_mode() &&
      control_conf_.control_test_duration() > 0 &&
      (start_time - init_time_).ToSecond() >
          control_conf_.control_test_duration()) {
    AERROR << "Control finished testing. exit";
    return false;
  }

  ControlCommand control_command;

  Status status = ProduceControlCommand(&control_command);
  AERROR_IF(!status.ok()) << "Failed to produce control command:"
                          << status.error_message();

  if (pad_received_) {
    control_command.mutable_pad_msg()->CopyFrom(pad_msg_);
    pad_received_ = false;
  }

  // forward estop reason among following control frames.
  if (estop_) {
    control_command.mutable_header()->mutable_status()->set_msg(estop_reason_);
  }

  // set header
  // 给控制消息打上时间戳
  control_command.mutable_header()->set_lidar_timestamp(
      local_view_.trajectory().header().lidar_timestamp());
  control_command.mutable_header()->set_camera_timestamp(
      local_view_.trajectory().header().camera_timestamp());
  control_command.mutable_header()->set_radar_timestamp(
      local_view_.trajectory().header().radar_timestamp());

  common::util::FillHeader(node_->Name(), &control_command);

  ADEBUG << control_command.ShortDebugString();
  if (control_conf_.is_control_test_mode()) {
    ADEBUG << "Skip publish control command in test mode";
    return true;
  }

  const auto end_time = Clock::Now();
  const double time_diff_ms = (end_time - start_time).ToSecond() * 1e3;
  ADEBUG << "total control time spend: " << time_diff_ms << " ms.";
  // total_time_ms, total_time_exceeded都是控制输出命令消息中的变量
  control_command.mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  // 设定控制模块超时
  control_command.mutable_latency_stats()->set_total_time_exceeded(
      time_diff_ms > control_conf_.control_period() * 1e3);
  ADEBUG << "control cycle time is: " << time_diff_ms << " ms.";
  status.Save(control_command.mutable_header()->mutable_status());

  // measure latency
  // 这个延迟是以轨迹中激光雷达的时间戳为准的吗？
  // 这个延迟和上边的超时有什么区别？分别有什么用？
  if (local_view_.trajectory().header().has_lidar_timestamp()) {
    static apollo::common::LatencyRecorder latency_recorder(
        FLAGS_control_command_topic);
    latency_recorder.AppendLatencyRecord(
        local_view_.trajectory().header().lidar_timestamp(), start_time,
        end_time);
  }

  control_cmd_writer_->Write(control_command);
  return true;
}

Status ControlComponent::CheckInput(LocalView *local_view) {
  ADEBUG << "Received localization:"
         << local_view->localization().ShortDebugString();
  ADEBUG << "Received chassis:" << local_view->chassis().ShortDebugString();
  // 轨迹没有紧急停车，并且轨迹是空的，拷贝消息队列号，返回错误信息
  if (!local_view->trajectory().estop().is_estop() &&
      local_view->trajectory().trajectory_point().empty()) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    const std::string msg =
        absl::StrCat("planning has no trajectory point. planning_seq_num:",
                     local_view->trajectory().header().sequence_num());
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, msg);
  }

  for (auto &trajectory_point :
       *local_view->mutable_trajectory()->mutable_trajectory_point()) {
    // 轨迹中点的速度小于速度最小分辨率，并且加速度小于停车时最大加速度
    // 表明车辆的速度很低了，几乎检测不到，而且车辆的制动也很小了，说明车辆接近静止
    // 这时把轨迹中点的速度和加速度都设为零，这有什么用？
    // 制动加速度是负数
    // 0.2(单位是m/s吗？), 0.01
    if (std::abs(trajectory_point.v()) <
            control_conf_.minimum_speed_resolution() &&
        std::abs(trajectory_point.a()) <
            control_conf_.max_acceleration_when_stopped()) {
      trajectory_point.set_v(0.0);
      trajectory_point.set_a(0.0);
    }
  }
  // 定位和底盘信息放在车辆状态中！
  injector_->vehicle_state()->Update(local_view->localization(),
                                     local_view->chassis());

  return Status::OK();
}

Status ControlComponent::CheckTimestamp(const LocalView &local_view) {
  // 未激活输入时间戳检测或者控制测试模式，不需要检查时间戳
  if (!control_conf_.enable_input_timestamp_check() ||
      control_conf_.is_control_test_mode()) {
    ADEBUG << "Skip input timestamp check by gflags.";
    return Status::OK();
  }
  double current_timestamp = Clock::NowInSeconds();
  // 当前时间戳减去拿到的定位消息的时间戳
  double localization_diff =
      current_timestamp - local_view.localization().header().timestamp_sec();
  // 定位数据最大丢包个数乘以定位消息发送周期
  // 20 * 0.01s = 0.2s
  if (localization_diff > (control_conf_.max_localization_miss_num() *
                           control_conf_.localization_period())) {
    AERROR << "Localization msg lost for " << std::setprecision(6)
           << localization_diff << "s";
    monitor_logger_buffer_.ERROR("Localization msg lost");
    // 消息丢包属于错误类型，要返回 控制计算错误 
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Localization msg timeout");
  }
  // 底盘消息丢包, 20 * 0.01 = 0.2s
  double chassis_diff =
      current_timestamp - local_view.chassis().header().timestamp_sec();
  if (chassis_diff >
      (control_conf_.max_chassis_miss_num() * control_conf_.chassis_period())) {
    AERROR << "Chassis msg lost for " << std::setprecision(6) << chassis_diff
           << "s";
    monitor_logger_buffer_.ERROR("Chassis msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Chassis msg timeout");
  }
  // 轨迹消息丢包, 20 * 0.1s = 2s
  double trajectory_diff =
      current_timestamp - local_view.trajectory().header().timestamp_sec();
  if (trajectory_diff > (control_conf_.max_planning_miss_num() *
                         control_conf_.trajectory_period())) {
    AERROR << "Trajectory msg lost for " << std::setprecision(6)
           << trajectory_diff << "s";
    monitor_logger_buffer_.ERROR("Trajectory msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Trajectory msg timeout");
  }
  return Status::OK();
}

}  // namespace control
}  // namespace apollo