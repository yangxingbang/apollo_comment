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

#include "modules/common/math/search.h"

#include <cmath>

namespace apollo {
namespace common {
namespace math {

// 函数声明中默认: tol = 1e-6
// 返回了下限和上限中间的某个值？
double GoldenSectionSearch(const std::function<double(double)> &func,
                           const double lower_bound, const double upper_bound,
                           const double tol) {
  
  static constexpr double gr = 1.618033989;  // (sqrt(5) + 1) / 2

  double a = lower_bound;
  double b = upper_bound;

  double t = (b - a) / gr;
  double c = b - t;
  double d = a + t;

  // 一直循环到c，d基本为同一s值，那么a，b之间相差2t
  while (std::abs(c - d) > tol) {
    // 改变上限
    if (func(c) < func(d)) {
      b = d;
    } else {
      a = c;
    }
    // 改变上限后再来计算一次c, d
    t = (b - a) / gr;
    c = b - t;
    d = a + t;
  }
  // 返回了a,b的平均值，为什么不直接返回，要做这个调整？
  return (a + b) * 0.5;
}

}  // namespace math
}  // namespace common
}  // namespace apollo
