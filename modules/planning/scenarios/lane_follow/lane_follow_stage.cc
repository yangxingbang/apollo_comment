/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file
 **/

#include "modules/planning/scenarios/lane_follow/lane_follow_stage.h"

#include <utility>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"
#include "modules/planning/tasks/deciders/path_decider/path_decider.h"
#include "modules/planning/tasks/deciders/speed_decider/speed_decider.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/path_time_heuristic_optimizer.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace lane_follow {

using apollo::common::ErrorCode;
using apollo::common::SLPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::util::PointFactory;
using apollo::cyber::Clock;

namespace {
constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

LaneFollowStage::LaneFollowStage(
    const ScenarioConfig::StageConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Stage(config, injector) {}

void LaneFollowStage::RecordObstacleDebugInfo(
    ReferenceLineInfo* reference_line_info) {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }
  auto ptr_debug = reference_line_info->mutable_debug();

  // PathDecision是一个类，表示当前路径上所有障碍物的决策，path_decision()就是调该类的对象
  // apollo中的决策都是通过比较障碍物类型的cost值和借道换道等的cost值来完成的吗？
  const auto path_decision = reference_line_info->path_decision();
  for (const auto obstacle : path_decision->obstacles().Items()) {
    auto obstacle_debug = ptr_debug->mutable_planning_data()->add_obstacle();
    obstacle_debug->set_id(obstacle->Id());
    // 拿sl的边界
    obstacle_debug->mutable_sl_boundary()->CopyFrom(
        obstacle->PerceptionSLBoundary());
    const auto& decider_tags = obstacle->decider_tags();
    // 进行纵向和横向的障碍物决策
    const auto& decisions = obstacle->decisions();
    if (decider_tags.size() != decisions.size()) {
      AERROR << "decider_tags size: " << decider_tags.size()
             << " different from decisions size:" << decisions.size();
    }
    // 给障碍物打上决策标签吗？如同在dreamview中看到的那样？
    for (size_t i = 0; i < decider_tags.size(); ++i) {
      auto decision_tag = obstacle_debug->add_decision_tag();
      decision_tag->set_decider_tag(decider_tags[i]);
      decision_tag->mutable_decision()->CopyFrom(decisions[i]);
    }
  }
}

// stage的process就是：
// 1. 根据参考线规划路径（task的执行在这里）
// 2. 确定路径的cost值
// 3. 做出是否换道的决策
Stage::StageStatus LaneFollowStage::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  bool has_drivable_reference_line = false;

  ADEBUG << "Number of reference lines:\t"
         << frame->mutable_reference_line_info()->size();

  unsigned int count = 0;

  // 遍历reference_line_info，导航涉及到的当前road的每个车道都会有一条参考线
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    // TODO(SHU): need refactor
    if (count++ == frame->mutable_reference_line_info()->size()) {
      break;
    }
    ADEBUG << "No: [" << count << "] Reference Line.";
    // 检查当前参考线是否变道的参考线
    ADEBUG << "IsChangeLanePath: " << reference_line_info.IsChangeLanePath();

    // 仅仅判断这个变量当前的状态
    // 确定它是否被不正当的改动了，如果改动了，就跳出
    if (has_drivable_reference_line) {
      reference_line_info.SetDrivable(false);
      break;
    }

    // 每个reference_line都规划路径，使用起始点和reference_line来规划
    auto cur_status =
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);

    // 换道决策器
    if (cur_status.ok()) {
      if (reference_line_info.IsChangeLanePath()) {
        ADEBUG << "reference line is lane change ref.";
        ADEBUG << "FLAGS_enable_smarter_lane_change: "
               << FLAGS_enable_smarter_lane_change;
        // 要换道，直行的cost值和换道前方干净要同时具备
        // enable_smarter_lane_change默认为负
        // 智能换道smarter_lane_change需要更长的换道距离，智能换道应该就是自动换道
        // kStraightForwardLineCost默认10
        if (reference_line_info.Cost() < kStraightForwardLineCost &&
            (LaneChangeDecider::IsClearToChangeLane(&reference_line_info) ||
             FLAGS_enable_smarter_lane_change)) {
          // If the path and speed optimization succeed on target lane while
          // under smart lane-change or IsClearToChangeLane under older version
          has_drivable_reference_line = true;
          reference_line_info.SetDrivable(true);
          // 能换道，更新换道的准备距离
          LaneChangeDecider::UpdatePreparationDistance(
              true, frame, &reference_line_info, injector_->planning_context());
          // \t表示水平制表位
          ADEBUG << "\tclear for lane change";
        } else {
          // 不能换道
          LaneChangeDecider::UpdatePreparationDistance(
              false, frame, &reference_line_info,
              injector_->planning_context());
          reference_line_info.SetDrivable(false);
          ADEBUG << "\tlane change failed";
        }
      } else {
        // 不能换道
        ADEBUG << "reference line is NOT lane change ref.";
        has_drivable_reference_line = true;
      }
    } else {
      // 根据参考线规划的结果状态不OK，规划失败
      reference_line_info.SetDrivable(false);
    }
  }

  // has_drivable_reference_line什么意思？
  return has_drivable_reference_line ? StageStatus::RUNNING
                                     : StageStatus::ERROR;
}

// 每个planning周期，参考线的cost会归0吗？
Status LaneFollowStage::PlanOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  if (!reference_line_info->IsChangeLanePath()) {
    // 如果不是换道，增大向前直行cost，cost初始值是0，默认增加10
    reference_line_info->AddCost(kStraightForwardLineCost);
  }
  ADEBUG << "planning start point:" << planning_start_point.DebugString();
  ADEBUG << "Current reference_line_info is IsChangeLanePath: "
         << reference_line_info->IsChangeLanePath();

  auto ret = Status::OK();
  // task_list_定义在父类里
  // 每个stage有多个task，task遍历执行，task分为决策器和规划器
  for (auto* task : task_list_) {
    const double start_timestamp = Clock::NowInSeconds();

    // Execute是虚函数，不是纯虚函数
    // 定义一个函数为虚函数，不代表函数为不被实现的函数。
    // 定义他为虚函数是为了允许用基类的指针来调用子类的这个函数。
    // 定义一个函数为纯虚函数，才代表函数没有被实现。
    // 定义纯虚函数是为了实现一个接口，起到一个规范的作用，规范继承这个类的程序员必须实现这个函数。
    ret = task->Execute(frame, reference_line_info);

    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
    ADEBUG << "after task[" << task->Name()
           << "]:" << reference_line_info->PathSpeedDebugString();
    ADEBUG << task->Name() << " time spend: " << time_diff_ms << " ms.";
    RecordDebugInfo(reference_line_info, task->Name(), time_diff_ms);

    if (!ret.ok()) {
      AERROR << "Failed to run tasks[" << task->Name()
             << "], Error message: " << ret.error_message();
      break;
    }

    // TODO(SHU): disable reference line order changes for now
    // updated reference_line_info, because it is changed in
    // lane_change_decider by PrioritizeChangeLane().
    // reference_line_info = &frame->mutable_reference_line_info()->front();
    // ADEBUG << "Current reference_line_info is IsChangeLanePath: "
    //        << reference_line_info->IsChangeLanePath();
  }

  RecordObstacleDebugInfo(reference_line_info);

  // check path and speed results for path or speed fallback
  reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);

  // 所有task遍历后，如果状态非OK，执行Fallback（应变计划，退路）
  if (!ret.ok()) {
    PlanFallbackTrajectory(planning_start_point, frame, reference_line_info);
  }

  DiscretizedTrajectory trajectory;
  // 把路径规划和速度规划的数据融合起来，对齐初始点的时间戳和s值
  if (!reference_line_info->CombinePathAndSpeedProfile(
          planning_start_point.relative_time(),
          planning_start_point.path_point().s(), &trajectory)) {
    const std::string msg = "Fail to aggregate planning trajectory.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // determine if there is a destination on reference line.
  // 障碍物决策
  double dest_stop_s = -1.0;
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    // 上边的决策得出结果是 has_stop， 而且停车的原因是到达目的地点
    if (obstacle->LongitudinalDecision().has_stop() &&
        obstacle->LongitudinalDecision().stop().reason_code() ==
            STOP_REASON_DESTINATION) {
      // 该障碍物引起的自车停车，这是已经决策完的结果，现在把停车点的坐标xy转为sl
      SLPoint dest_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                  reference_line_info->reference_line());
      dest_stop_s = dest_sl.s();
    }
  }

  // 又是障碍物决策，这和上面不重复吗？
  // 除过virtual和staic的判断，剩余逻辑就是重复的！
  // 
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (!obstacle->IsStatic()) {
      continue;
    }
    if (obstacle->LongitudinalDecision().has_stop()) {
      bool add_stop_obstacle_cost = false;
      if (dest_stop_s < 0.0) {
        add_stop_obstacle_cost = true;
      } else {
        SLPoint stop_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                    reference_line_info->reference_line());
        if (stop_sl.s() < dest_stop_s) {
          add_stop_obstacle_cost = true;
        }
      }
      if (add_stop_obstacle_cost) {
        // 静态障碍物参考线cost值默认1000
        static constexpr double kReferenceLineStaticObsCost = 1e3;
        // 发现静态障碍物，就+1000
        reference_line_info->AddCost(kReferenceLineStaticObsCost);
      }
    }
  }

  // 轨迹检查
  if (FLAGS_enable_trajectory_check) {
    if (ConstraintChecker::ValidTrajectory(trajectory) !=
        ConstraintChecker::Result::VALID) {
      const std::string msg = "Current planning trajectory is not valid.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  reference_line_info->SetTrajectory(trajectory);
  reference_line_info->SetDrivable(true);
  return Status::OK();
}

// 看到此处！
void LaneFollowStage::PlanFallbackTrajectory(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  // path and speed fall back

  // 如果没能生成轨迹，那就沿着当前车辆行驶方向生成向前100m的轨迹
  // 然后增大路径优化fallback的cost值，默认是20000，这个值很大！
  if (reference_line_info->path_data().Empty()) {
    AERROR << "Path fallback due to algorithm failure";
    GenerateFallbackPathProfile(reference_line_info,
                                reference_line_info->mutable_path_data());
    reference_line_info->AddCost(kPathOptimizationFallbackCost);
    reference_line_info->set_trajectory_type(ADCTrajectory::PATH_FALLBACK);
  }

  // 路径轨迹的类型有：UNKNOWN，NORMAL，PATH_FALLBACK，SPEED_FALLBACK，PATH_REUSED
  // path和speed都有fallback
  if (reference_line_info->trajectory_type() != ADCTrajectory::PATH_FALLBACK) {
    // 检索上帧的路径
    if (!RetrieveLastFramePathProfile(
            reference_line_info, frame,
            reference_line_info->mutable_path_data())) {
      const auto& candidate_path_data =
          reference_line_info->GetCandidatePathData();
      for (const auto& path_data : candidate_path_data) {
        if (path_data.path_label().find("self") != std::string::npos) {
          *reference_line_info->mutable_path_data() = path_data;
          AERROR << "Use current frame self lane path as fallback ";
          break;
        }
      }
    }
  }

  AERROR << "Speed fallback due to algorithm failure";
  *reference_line_info->mutable_speed_data() =
      SpeedProfileGenerator::GenerateFallbackSpeed(injector_->ego_info());

  if (reference_line_info->trajectory_type() != ADCTrajectory::PATH_FALLBACK) {
    reference_line_info->AddCost(kSpeedOptimizationFallbackCost);
    reference_line_info->set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);
  }
}

void LaneFollowStage::GenerateFallbackPathProfile(
    const ReferenceLineInfo* reference_line_info, PathData* path_data) {
  const double unit_s = 1.0;
  const auto& reference_line = reference_line_info->reference_line();

  auto adc_point = injector_->ego_info()->start_point();
  DCHECK(adc_point.has_path_point());
  const auto adc_point_x = adc_point.path_point().x();
  const auto adc_point_y = adc_point.path_point().y();

  common::SLPoint adc_point_s_l;
  // 这种投影不成功的问题是怎么产生的？
  if (!reference_line.XYToSL(adc_point.path_point(), &adc_point_s_l)) {
    AERROR << "Fail to project ADC to reference line when calculating path "
              "fallback. Straight forward path is generated";
    const auto adc_point_heading = adc_point.path_point().theta();
    const auto adc_point_kappa = adc_point.path_point().kappa();
    const auto adc_point_dkappa = adc_point.path_point().dkappa();
    std::vector<common::PathPoint> path_points;
    double adc_traversed_x = adc_point_x;
    double adc_traversed_y = adc_point_y;

    const double max_s = 100.0;
    for (double s = 0; s < max_s; s += unit_s) {
      path_points.push_back(PointFactory::ToPathPoint(
          adc_traversed_x, adc_traversed_y, 0.0, s, adc_point_heading,
          adc_point_kappa, adc_point_dkappa));
      adc_traversed_x += unit_s * std::cos(adc_point_heading);
      adc_traversed_y += unit_s * std::sin(adc_point_heading);
    }
    path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
    return;
  }

  // Generate a fallback path along the reference line direction
  const auto adc_s = adc_point_s_l.s();
  const auto& adc_ref_point =
      reference_line.GetReferencePoint(adc_point_x, adc_point_y);
  const double dx = adc_point_x - adc_ref_point.x();
  const double dy = adc_point_y - adc_ref_point.y();

  std::vector<common::PathPoint> path_points;
  const double max_s = reference_line.Length();
  for (double s = adc_s; s < max_s; s += unit_s) {
    const auto& ref_point = reference_line.GetReferencePoint(s);
    path_points.push_back(PointFactory::ToPathPoint(
        ref_point.x() + dx, ref_point.y() + dy, 0.0, s - adc_s,
        ref_point.heading(), ref_point.kappa(), ref_point.dkappa()));
  }
  path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
}

bool LaneFollowStage::RetrieveLastFramePathProfile(
    const ReferenceLineInfo* reference_line_info, const Frame* frame,
    PathData* path_data) {
  const auto* ptr_last_frame = injector_->frame_history()->Latest();
  if (ptr_last_frame == nullptr) {
    AERROR
        << "Last frame doesn't succeed, fail to retrieve last frame path data";
    return false;
  }

  const auto& last_frame_discretized_path =
      ptr_last_frame->current_frame_planned_path();

  path_data->SetDiscretizedPath(last_frame_discretized_path);
  const auto adc_frenet_frame_point_ =
      reference_line_info->reference_line().GetFrenetPoint(
          frame->PlanningStartPoint().path_point());

  bool trim_success = path_data->LeftTrimWithRefS(adc_frenet_frame_point_);
  if (!trim_success) {
    AERROR << "Fail to trim path_data. adc_frenet_frame_point: "
           << adc_frenet_frame_point_.ShortDebugString();
    return false;
  }
  AERROR << "Use last frame good path to do speed fallback";
  return true;
}

SLPoint LaneFollowStage::GetStopSL(const ObjectStop& stop_decision,
                                   const ReferenceLine& reference_line) const {
  SLPoint sl_point;
  // 把障碍物引起的停车点的坐标进行转换
  reference_line.XYToSL(stop_decision.stop_point(), &sl_point);
  return sl_point;
}

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
