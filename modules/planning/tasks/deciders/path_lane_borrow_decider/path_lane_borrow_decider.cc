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

#include "modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_decider.h"

#include <algorithm>
#include <memory>
#include <string>

#include "modules/planning/common/obstacle_blocking_analyzer.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

// 交叉路口干净路段的距离
constexpr double kIntersectionClearanceDist = 20.0;
// 丁字路口干净路段的距离
constexpr double kJunctionClearanceDist = 15.0;

// 有参构造函数体内没有实现，值得注意！
PathLaneBorrowDecider::PathLaneBorrowDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {}

Status PathLaneBorrowDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // skip path_lane_borrow_decider if reused path
  // 假如路径复用上一帧的路径话，跳过借道决策器
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
    // for debug
    AINFO << "skip due to reusing path";
    return Status::OK();
  }

  // By default, don't borrow any lane.
  // 默认不借道
  reference_line_info->set_is_path_lane_borrow(false);
  // Check if lane-borrowing is needed, if so, borrow lane.
  if (Decider::config_.path_lane_borrow_decider_config()
          .allow_lane_borrowing() &&
      IsNecessaryToBorrowLane(*frame, *reference_line_info)) {
    reference_line_info->set_is_path_lane_borrow(true);
  }
  return Status::OK();
}

bool PathLaneBorrowDecider::IsNecessaryToBorrowLane(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  // is_in_path_lane_borrow_scenario处于借道场景
  if (mutable_path_decider_status->is_in_path_lane_borrow_scenario()) {
    // If originally borrowing neighbor lane:
    // 一开始要借道，但是过了不久，发现走自己车道也能过去，就不借道
    if (mutable_path_decider_status->able_to_use_self_lane_counter() >= 6) {
      // If have been able to use self-lane for some time, then switch to
      // non-lane-borrowing.
      mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
      // 清除已决策好的侧向通行方向，即，把之前决策好的借道的方向清了
      mutable_path_decider_status->clear_decided_side_pass_direction();
      AINFO << "Switch from LANE-BORROW path to SELF-LANE path.";
    }
  } else {
    // If originally not borrowing neighbor lane:
    // 默认的参数导致一开始不处于借道场景
    ADEBUG << "Blocking obstacle ID["
           << mutable_path_decider_status->front_static_obstacle_id() << "]";
    // ADC requirements check for lane-borrowing:
    // 只有一条参考线
    if (!HasSingleReferenceLine(frame)) {
      return false;
    }
    // 规划的借道轨迹起点车速低于限值5m/s=18kph
    if (!IsWithinSidePassingSpeedADC(frame)) {
      return false;
    }

    // Obstacle condition check for lane-borrowing:
    // 障碍物距离交叉路口很远
    if (!IsBlockingObstacleFarFromIntersection(reference_line_info)) {
      return false;
    }
    // 静态障碍物长期占用车道，演示项目的绕道就是这个case
    if (!IsLongTermBlockingObstacle()) {
      return false;
    }
    // 目的地被障碍物堵塞
    if (!IsBlockingObstacleWithinDestination(reference_line_info)) {
      return false;
    }
    // 应该侧绕过的障碍物
    if (!IsSidePassableObstacle(reference_line_info)) {
      return false;
    }

    // switch to lane-borrowing
    // set side-pass direction
    // 一开始就不能借道，然后检查是否可以借道，如果能借，就往左或右借道
    const auto& path_decider_status =
        injector_->planning_context()->planning_status().path_decider();
    if (path_decider_status.decided_side_pass_direction().empty()) {
      // first time init decided_side_pass_direction
      bool left_borrowable;
      bool right_borrowable;
      CheckLaneBorrow(reference_line_info, &left_borrowable, &right_borrowable);
      if (!left_borrowable && !right_borrowable) {
        mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
        return false;
      } else {
        // 左边借道或右边借道有一个能成功，就判定可以进入借道场景，然后决策为借道
        mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
        if (left_borrowable) {
          mutable_path_decider_status->add_decided_side_pass_direction(
              PathDeciderStatus::LEFT_BORROW);
        }
        if (right_borrowable) {
          mutable_path_decider_status->add_decided_side_pass_direction(
              PathDeciderStatus::RIGHT_BORROW);
        }
      }
    }

    AINFO << "Switch from SELF-LANE path to LANE-BORROW path.";
  }
  return mutable_path_decider_status->is_in_path_lane_borrow_scenario();
}

// This function is to prevent lane-borrowing during lane-changing.
// TODO(jiacheng): depending on our needs, may allow lane-borrowing during
//                 lane-change.
bool PathLaneBorrowDecider::HasSingleReferenceLine(const Frame& frame) {
  return frame.reference_line_info().size() == 1;
}

// 规划的借道起始点的车速如果大于5mps，不允许借道
bool PathLaneBorrowDecider::IsWithinSidePassingSpeedADC(const Frame& frame) {
  return frame.PlanningStartPoint().v() < FLAGS_lane_borrow_max_speed;
}

// 长时间(大于3s)阻塞车道的障碍物
bool PathLaneBorrowDecider::IsLongTermBlockingObstacle() {
  if (injector_->planning_context()
          ->planning_status()
          .path_decider()
          .front_static_obstacle_cycle_counter() >=
      FLAGS_long_term_blocking_obstacle_cycle_threshold) {
    ADEBUG << "The blocking obstacle is long-term existing.";
    return true;
  } else {
    ADEBUG << "The blocking obstacle is not long-term existing.";
    return false;
  }
}

// 阻塞目的地的障碍物
bool PathLaneBorrowDecider::IsBlockingObstacleWithinDestination(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }

  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().start_s();
  double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
  ADEBUG << "ADC is at s = " << adc_end_s;
  ADEBUG << "Destination is at s = "
         << reference_line_info.SDistanceToDestination() + adc_end_s;
  if (blocking_obstacle_s - adc_end_s >
      reference_line_info.SDistanceToDestination()) {
    return false;
  }
  return true;
}

bool PathLaneBorrowDecider::IsBlockingObstacleFarFromIntersection(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }

  // Get blocking obstacle's s.
  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().end_s();
  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
  // Get intersection's s and compare with threshold.
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  for (const auto& overlap : first_encountered_overlaps) {
    ADEBUG << overlap.first << ", " << overlap.second.DebugString();
    // if (// overlap.first != ReferenceLineInfo::CLEAR_AREA &&
    // overlap.first != ReferenceLineInfo::CROSSWALK &&
    // overlap.first != ReferenceLineInfo::PNC_JUNCTION &&
    if (overlap.first != ReferenceLineInfo::SIGNAL &&
        overlap.first != ReferenceLineInfo::STOP_SIGN) {
      continue;
    }

    auto distance = overlap.second.start_s - blocking_obstacle_s;
    if (overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::STOP_SIGN) {
      if (distance < kIntersectionClearanceDist) {
        ADEBUG << "Too close to signal intersection (" << distance
               << "m); don't SIDE_PASS.";
        return false;
      }
    } else {
      if (distance < kJunctionClearanceDist) {
        ADEBUG << "Too close to overlap_type[" << overlap.first << "] ("
               << distance << "m); don't SIDE_PASS";
        return false;
      }
    }
  }

  return true;
}

bool PathLaneBorrowDecider::IsSidePassableObstacle(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return false;
  }
  // path_decision表示了同一个路径上所有的障碍物决策
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return false;
  }

  // 障碍物是不可移动的时候，就侧向通过
  return IsNonmovableObstacle(reference_line_info, *blocking_obstacle);
}

void PathLaneBorrowDecider::CheckLaneBorrow(
    const ReferenceLineInfo& reference_line_info,
    bool* left_neighbor_lane_borrowable, bool* right_neighbor_lane_borrowable) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  *left_neighbor_lane_borrowable = true;
  *right_neighbor_lane_borrowable = true;

  // 对于修饰Object来说，const并未区分出编译期常量和运行期常量
  // constexpr限定在了编译期常量
  // 把原本运行期做的事情搬到编译期去做
  static constexpr double kLookforwardDistance = 100.0;
  double check_s = reference_line_info.AdcSlBoundary().end_s();
  const double lookforward_distance =
      std::min(check_s + kLookforwardDistance, reference_line.Length());
  // 把自车的sl边界投影到参考线上
  while (check_s < lookforward_distance) {
    auto ref_point = reference_line.GetNearestReferencePoint(check_s);
    if (ref_point.lane_waypoints().empty()) {
      *left_neighbor_lane_borrowable = false;
      *right_neighbor_lane_borrowable = false;
      return;
    }

    const auto waypoint = ref_point.lane_waypoints().front();
    hdmap::LaneBoundaryType::Type lane_boundary_type =
        hdmap::LaneBoundaryType::UNKNOWN;

    // 借道用到 黄实线 和 白实线 来判断
    if (*left_neighbor_lane_borrowable) {
      lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
      if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
          lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
        *left_neighbor_lane_borrowable = false;
      }
      ADEBUG << "s[" << check_s << "] left_lane_boundary_type["
             << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
    }
    if (*right_neighbor_lane_borrowable) {
      lane_boundary_type = hdmap::RightBoundaryType(waypoint);
      if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
          lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
        *right_neighbor_lane_borrowable = false;
      }
      ADEBUG << "s[" << check_s << "] right_neighbor_lane_borrowable["
             << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
    }
    check_s += 2.0;
  }
}

}  // namespace planning
}  // namespace apollo
