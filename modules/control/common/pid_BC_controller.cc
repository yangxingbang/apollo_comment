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

#include "modules/control/common/pid_BC_controller.h"

#include <cmath>

#include "cyber/common/log.h"

#include "modules/common/math/math_utils.h"

namespace apollo {
namespace control {

double PIDBCController::Control(const double error, const double dt) {
  if (dt <= 0) {
    AWARN << "dt <= 0, will use the last output";
    return previous_output_;
  }
  double diff = 0;
  double output = 0;

  if (first_hit_) {
    first_hit_ = false;
  } else {
    diff = (error - previous_error_) / dt;
  }

  // backward calculation
  if (!integrator_enabled_) {
    integral_ = 0;
  } else {
    // 公式实际上与pid没有不同
    double u = error * kp_ + integral_ + error * dt * ki_ + diff * kd_;
    // 经饱和限幅后的输出减去没有限幅的输出，结果作为反馈项，将会反馈到积分项的计算中
    double aw_term = common::math::Clamp(u, output_saturation_high_,
                                         output_saturation_low_) - u;
    // 这里的判定标准值太小了，能有什么效果，只会导致aw_term符号不断变化？
    if (aw_term > 1e-6) { // // 表示限幅后的值，大于pid结果
      output_saturation_status_ = -1;
    } else if (aw_term < -1e-6) { // 表示限幅后的值，小于pid结果
      output_saturation_status_ = 1;
    } else {
      output_saturation_status_ = 0;
    }
    // 反馈抑制饱和（参见文档），kaw_ * aw_term为反馈项，它的范围为0.3~3*Ki/Kp
    // 目前站位pid里kaw_是0，速度pid里kaw_是1.0
    integral_ += kaw_ * aw_term + error * dt;
  }

  previous_error_ = error;
  output = common::math::Clamp(error * kp_ + integral_ + diff * kd_,
                               output_saturation_high_,
                               output_saturation_low_);  // Ki already applied
  previous_output_ = output;
  return output;
}

int PIDBCController::OutputSaturationStatus() {
  return output_saturation_status_;
}

}  // namespace control
}  // namespace apollo
