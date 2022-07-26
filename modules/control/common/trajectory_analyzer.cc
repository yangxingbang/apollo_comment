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

#include "modules/control/common/trajectory_analyzer.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "Eigen/Core"

#include "cyber/common/log.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/search.h"
#include "modules/control/common/control_gflags.h"

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;

namespace apollo {
namespace control {
namespace {

// Squared distance from the point to (x, y).
double PointDistanceSquare(const TrajectoryPoint &point, const double x,
                           const double y) {
  const double dx = point.path_point().x() - x;
  const double dy = point.path_point().y() - y;
  return dx * dx + dy * dy;
}

PathPoint TrajectoryPointToPathPoint(const TrajectoryPoint &point) {
  if (point.has_path_point()) {
    return point.path_point();
  } else {
    // pnc_point.proto中的PathPoint消息，有消息格式，但是没有数值，数值在哪儿？
    return PathPoint();
  }
}

}  // namespace

// 注意，以下函数均不在control命名空间中，属于全局函数！

TrajectoryAnalyzer::TrajectoryAnalyzer(
    const planning::ADCTrajectory *planning_published_trajectory) {
  header_time_ = planning_published_trajectory->header().timestamp_sec();
  seq_num_ = planning_published_trajectory->header().sequence_num();

  for (int i = 0; i < planning_published_trajectory->trajectory_point_size();
       ++i) {
    trajectory_points_.push_back(
        planning_published_trajectory->trajectory_point(i));
  }
}

// 这个函数返回一个“虚拟点”的数据
PathPoint TrajectoryAnalyzer::QueryMatchedPathPoint(const double x,
                                                    const double y) const {
  //  GT: greater than 大于等于
  CHECK_GT(trajectory_points_.size(), 0);
  // 轨迹上第一个点到车辆当前位置的几何距离，坐标系变换不会改变距离大小
  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;
  // 从轨迹的第二个点开始，遍历所有点，找到距离车辆位置最近的点
  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  // 判断距车最近点的索引是不是轨迹的起始点，它的下一个点是不是轨迹的终点
  // 如果最小距离点是第一个点，把第一个作为index_start
  // 如果最小距离点不是第一个点，把它之前（靠近车）的一个点作为index_start
  size_t index_start = index_min == 0 ? index_min : index_min - 1;
  // 如果最小距离点的下一个点（远离车）是轨迹最后一个点，把最小距离点作为index_end
  // 如果最小距离点的下一个点（远离车）不是轨迹最后一个点，把最小距离点的下一个点作为index_end
  size_t index_end =
      index_min + 1 == trajectory_points_.size() ? index_min : index_min + 1;
  // 所以最普通的情况下，index_end和index_end相差2个点

  const double kEpsilon = 0.001;
  // 表示轨迹只有一个点，或者index_end和index_end距离很小
  // 相当于index_start和index_end是同一个点，返回该点的路径点消息
  if (index_start == index_end ||
      std::fabs(trajectory_points_[index_start].path_point().s() -
                trajectory_points_[index_end].path_point().s()) <= kEpsilon) {
    return TrajectoryPointToPathPoint(trajectory_points_[index_start]);
  }
  // 通常情况下，index_end和index_end不会是同一个点，他们之间还有一个点
  // 在index_end和index_end之间插值，（利用复杂的规则）找到虚拟的距离车辆当前位置最近的点
  // 并返回该路径点的数据
  return FindMinDistancePoint(trajectory_points_[index_start],
                              trajectory_points_[index_end], x, y);
}

// 参考轨迹定义在frenet坐标系，从车辆当前位置到参考点航向的连线与航向垂直

// reference: Optimal trajectory generation for dynamic street scenarios in a
// Frenét Frame,
// Moritz Werling, Julius Ziegler, Sören Kammel and Sebastian Thrun, ICRA 2010
// similar to the method in this paper without the assumption the "normal"
// vector
// (from vehicle position to ref_point position) and reference heading are
// perpendicular.

void TrajectoryAnalyzer::ToTrajectoryFrame(const double x, const double y,
                                           const double theta, const double v,
                                           const PathPoint &ref_point,
                                           double *ptr_s, double *ptr_s_dot,
                                           double *ptr_d,
                                           double *ptr_d_dot) const {
  double dx = x - ref_point.x();
  double dy = y - ref_point.y();
  // 虚拟点的航向角的三角函数分量
  double cos_ref_theta = std::cos(ref_point.theta());
  double sin_ref_theta = std::sin(ref_point.theta());

  // the sin of diff angle between vector (cos_ref_theta, sin_ref_theta) and
  // (dx, dy)
  // cross_rd_nd应该是L方向的距离差
  double cross_rd_nd = cos_ref_theta * dy - sin_ref_theta * dx;
  *ptr_d = cross_rd_nd;

  // the cos of diff angle between vector (cos_ref_theta, sin_ref_theta) and
  // (dx, dy)
  // dot_rd_nd应该是S方向的距离差
  double dot_rd_nd = dx * cos_ref_theta + dy * sin_ref_theta;
  // frenet坐标系下，车辆当前位置的s值
  *ptr_s = ref_point.s() + dot_rd_nd;

  // 全局坐标系下，车辆当前位置的航向角和虚拟点航向角的差
  double delta_theta = theta - ref_point.theta();
  double cos_delta_theta = std::cos(delta_theta);
  double sin_delta_theta = std::sin(delta_theta);

  // frenet坐标系下车辆的横向速度分量
  *ptr_d_dot = v * sin_delta_theta;

  // 该变量是在公式推导中，利用“Frenet公式”来求微分时，自然产生的
  // 并不是开发者自己定义的
  double one_minus_kappa_r_d = 1 - ref_point.kappa() * (*ptr_d);
  if (one_minus_kappa_r_d <= 0.0) {
    AERROR << "TrajectoryAnalyzer::ToTrajectoryFrame "
              "found fatal reference and actual difference. "
              "Control output might be unstable:"
           << " ref_point.kappa:" << ref_point.kappa()
           << " ref_point.x:" << ref_point.x()
           << " ref_point.y:" << ref_point.y() << " car x:" << x
           << " car y:" << y << " *ptr_d:" << *ptr_d
           << " one_minus_kappa_r_d:" << one_minus_kappa_r_d;
    // currently set to a small value to avoid control crash.
    one_minus_kappa_r_d = 0.01;
  }

  // frenet坐标系下车辆的纵向速度分量
  *ptr_s_dot = v * cos_delta_theta / one_minus_kappa_r_d;
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByAbsoluteTime(
    const double t) const {
  // 绝对时间 = 当前时刻 + 搜索时长
  // 相对时间 = 绝对时间 - 减去轨迹信息时间戳
  // 这里的轨迹时间戳，指这一段轨迹开始时候的时刻
  // 
  return QueryNearestPointByRelativeTime(t - header_time_);
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByRelativeTime(
    const double t) const {
  // 这里的t，就是相对时间，是某个时间戳与轨迹时间戳的差
  // 回调函数返回的是 “某个轨迹点的相对时间“ 和 “形参相对时间“ 的比较结果
  // 从哪儿传参的？
  auto func_comp = [](const TrajectoryPoint &point,
                      const double relative_time) {
    return point.relative_time() < relative_time;
  };

  // 从起始点到终点逐个比较每个点的时间戳和t对应点时间戳的大小，来确定t对应点在轨迹上的相对位置
  auto it_low = std::lower_bound(trajectory_points_.begin(),
                                 trajectory_points_.end(), t, func_comp);

  // 假如最小时间是第一个轨迹点，返回第一个点
  // 这表明t对应的点在第一个点前边（远离车）
  if (it_low == trajectory_points_.begin()) {
    return trajectory_points_.front();
  }

  // 假如最小时间是最后一个轨迹点，返回最后一个点
  // 这表明t对应的点在最后一个轨迹点的前边（远离车的方向）
  // trajectory_points_.begin()还会大于trajectory_points_.end()吗？多余校验！
  if (it_low == trajectory_points_.end()) {
    return trajectory_points_.back();
  }

  // 搜索前向时间点，初始化为假，并在全文中未修改，所以这个判断只执行else分支
  if (FLAGS_query_forward_time_point_only) {
    return *it_low;
  } else {
    auto it_lower = it_low - 1;
    // 这里在判断，找到的这个点在时间上，是距it_low更近，还是距t更近
    // 返回更近的那个点
    if (it_low->relative_time() - t < t - it_lower->relative_time()) {
      return *it_low;
    }
    return *it_lower;
  }
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByPosition(
    const double x, const double y) const {
  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;

  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  return trajectory_points_[index_min];
}

const std::vector<TrajectoryPoint> &TrajectoryAnalyzer::trajectory_points()
    const {
  return trajectory_points_;
}

PathPoint TrajectoryAnalyzer::FindMinDistancePoint(const TrajectoryPoint &p0,
                                                   const TrajectoryPoint &p1,
                                                   const double x,
                                                   const double y) const {
  // given the fact that the discretized trajectory is dense enough,
  // we assume linear trajectory between consecutive trajectory points.
  // 线性差值以后再求距离平方
  auto dist_square = [&p0, &p1, &x, &y](const double s) {
    // 插值出某个s对应的x和y，s是从轨迹第一个点算起到指定位置的弧长
    double px = common::math::lerp(p0.path_point().x(), p0.path_point().s(),
                                   p1.path_point().x(), p1.path_point().s(), s);
    double py = common::math::lerp(p0.path_point().y(), p0.path_point().s(),
                                   p1.path_point().y(), p1.path_point().s(), s);
    double dx = px - x;
    double dy = py - y;
    return dx * dx + dy * dy;
  };

  PathPoint p = p0.path_point();
  // 返回了p0和p1之间的某个虚拟点的s值
  double s = common::math::GoldenSectionSearch(dist_square, p0.path_point().s(),
                                               p1.path_point().s());
  p.set_s(s);
  // 然后插值得这个虚拟点的多个物理量
  p.set_x(common::math::lerp(p0.path_point().x(), p0.path_point().s(),
                             p1.path_point().x(), p1.path_point().s(), s));
  p.set_y(common::math::lerp(p0.path_point().y(), p0.path_point().s(),
                             p1.path_point().y(), p1.path_point().s(), s));
  p.set_theta(common::math::slerp(p0.path_point().theta(), p0.path_point().s(),
                                  p1.path_point().theta(), p1.path_point().s(),
                                  s));
  // approximate the curvature at the intermediate point
  p.set_kappa(common::math::lerp(p0.path_point().kappa(), p0.path_point().s(),
                                 p1.path_point().kappa(), p1.path_point().s(),
                                 s));
  return p;
}

void TrajectoryAnalyzer::TrajectoryTransformToCOM(
    const double rear_to_com_distance) {
  CHECK_GT(trajectory_points_.size(), 0);
  for (size_t i = 0; i < trajectory_points_.size(); ++i) {
    auto com = ComputeCOMPosition(rear_to_com_distance,
                                  trajectory_points_[i].path_point());
    trajectory_points_[i].mutable_path_point()->set_x(com.x());
    trajectory_points_[i].mutable_path_point()->set_y(com.y());
  }
}

common::math::Vec2d TrajectoryAnalyzer::ComputeCOMPosition(
    const double rear_to_com_distance, const PathPoint &path_point) const {
  // Initialize the vector for coordinate transformation of the position
  // reference point
  Eigen::Vector3d v;
  const double cos_heading = std::cos(path_point.theta());
  const double sin_heading = std::sin(path_point.theta());
  v << rear_to_com_distance * cos_heading, rear_to_com_distance * sin_heading,
      0.0;
  // Original position reference point at center of rear-axis
  Eigen::Vector3d pos_vec(path_point.x(), path_point.y(), path_point.z());
  // Transform original position with vector v
  Eigen::Vector3d com_pos_3d = v + pos_vec;
  // Return transfromed x and y
  return common::math::Vec2d(com_pos_3d[0], com_pos_3d[1]);
}

}  // namespace control
}  // namespace apollo
