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

#include "modules/common/filters/digital_filter.h"

#include <cmath>

#include "cyber/common/log.h"

namespace {

const double kDoubleEpsilon = 1.0e-6;

}  // namespace

namespace apollo {
namespace common {

DigitalFilter::DigitalFilter(const std::vector<double> &denominators,
                             const std::vector<double> &numerators) {
  set_coefficients(denominators, numerators);
}

void DigitalFilter::set_denominators(const std::vector<double> &denominators) {
  denominators_ = denominators;
  y_values_.resize(denominators_.size(), 0.0);
}

void DigitalFilter::set_numerators(const std::vector<double> &numerators) {
  numerators_ = numerators;
  x_values_.resize(numerators_.size(), 0.0);
}

void DigitalFilter::set_coefficients(const std::vector<double> &denominators,
                                     const std::vector<double> &numerators) {
  set_denominators(denominators);
  set_numerators(numerators);
}

// 该函数默认只在单元测试中被调用
void DigitalFilter::set_dead_zone(const double deadzone) {
  dead_zone_ = std::fabs(deadzone);
  AINFO << "Setting digital filter dead zone = " << dead_zone_;
}

// 公式定义在digital_filter.h中
double DigitalFilter::Filter(const double x_insert) {
  if (denominators_.empty() || numerators_.empty()) {
    AERROR << "Empty denominators or numerators";
    return 0.0;
  }

  // 每次在做求和以前，先把数据更新一下
  // 把双端队列x的最旧值出队，双端队列x的最新值入队
  x_values_.pop_back();
  x_values_.push_front(x_insert);
  // 把分子各项求和
  const double xside =
      Compute(x_values_, numerators_, 0, numerators_.size() - 1);

  // 每次在做求和以前，先把数据更新一下
  // 把双端队列y的最旧值出队，然后把计算出的最新值入队，为下一个时间步计算做准备
  y_values_.pop_back();
  // 把分母各项求和
  const double yside =
      Compute(y_values_, denominators_, 1, denominators_.size() - 1);

  double y_insert = 0.0;
  // 分母的第一个系数项有值的时候，为何做此操作？
  if (std::fabs(denominators_.front()) > kDoubleEpsilon) {
    y_insert = (xside - yside) / denominators_.front();
  }
  y_values_.push_front(y_insert);

  return UpdateLast(y_insert);
}

void DigitalFilter::reset_values() {
  std::fill(x_values_.begin(), x_values_.end(), 0.0);
  std::fill(y_values_.begin(), y_values_.end(), 0.0);
}

// 前后两个时间步的值相差太小，达不到滤波器的分辨率
double DigitalFilter::UpdateLast(const double input) {
  const double diff = std::fabs(input - last_);
  if (diff < dead_zone_) {
    return last_;
  }
  last_ = input;
  return input;
}

double DigitalFilter::Compute(const std::deque<double> &values,
                              const std::vector<double> &coefficients,
                              const std::size_t coeff_start,
                              const std::size_t coeff_end) {
  ACHECK(coeff_start <= coeff_end && coeff_end < coefficients.size());
  ACHECK((coeff_end - coeff_start + 1) == values.size());

  double sum = 0.0;
  auto i = coeff_start;
  for (const auto value : values) {
    sum += value * coefficients[i];
    ++i;
  }
  return sum;
}

const std::vector<double> &DigitalFilter::denominators() const {
  return denominators_;
}

const std::vector<double> &DigitalFilter::numerators() const {
  return numerators_;
}

double DigitalFilter::dead_zone() const { return dead_zone_; }

const std::deque<double> &DigitalFilter::inputs_queue() const {
  return x_values_;
}

const std::deque<double> &DigitalFilter::outputs_queue() const {
  return y_values_;
}

}  // namespace common
}  // namespace apollo
