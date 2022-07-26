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

#include "modules/control/common/mrac_controller.h"

#include <cmath>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/LU"

#include "absl/strings/str_cat.h"
#include "cyber/common/log.h"
#include "modules/common/math/matrix_operations.h"

namespace apollo {
namespace control {

using apollo::common::ErrorCode;
using apollo::common::LatencyParam;
using apollo::common::Status;
using Matrix = Eigen::MatrixXd;

double MracController::Control(const double command, const Matrix state,
                               const double input_limit,
                               const double input_rate_limit) {
  // check if the current sampling time is valid and the reference/adaption
  // model well set up during the initialization
  if (ts_ <= 0.0 || !reference_model_enabled_ || !adaption_model_enabled_) {
    AERROR << "MRAC: model build failed; will work as a unity compensator. The "
              "reference_model building status: "
           << reference_model_enabled_
           << "; The adaption_model building status: "
           << adaption_model_enabled_ << "; Current sampling time: " << ts_;
    return command;  // treat the mrac as a unity proportional controller
  }

  // update the state in the real actuation system
  // 车辆底盘反馈的当前时间步的转角和转角速度
  state_action_.col(0) = state;

  // update the desired command in the real actuation system
  // LQR输出的方向盘转角百分比
  input_desired_(0, 0) = command;

  // update the command bounds for the real actuation system
  // 在物理极限的基础上再做限幅，默认值1，可给小于1的数
  bound_command_ = input_limit * bound_ratio_;
  bound_command_rate_ = input_rate_limit * bound_ratio_;

  // update the state in the reference system
  // 数值计算当前时间步的参考模型的状态变量
  UpdateReference();

  double state_bounded = 0.0;
  // mrac的参考模型的状态量（转角，转角速度）的限幅滤波，得到状态量
  saturation_status_reference_ = BoundOutput(
      state_reference_(0, 0), state_reference_(0, 1), &state_bounded);
  state_reference_(0, 0) = state_bounded;

  // update the adaption laws including state adaption, command adaption and
  // nonlinear components adaption

  // gamma_ratio_state_是一个代码中定义的标量，并不存在于公式中
  // SetStateAdaptionRate函数称它为“”自适应状态的变化率“，并为它赋值1或0
  // 赋0的时候，就不再更新，相当于不再启用自适应控制
  // gamma_state_adaption_是自适应律微分方程中的速率Gamma_x， 对于二阶系统它就是二阶方阵
  // 在这里代入自适应律微分方程中的Gamma_x被乘上了一个系数
  // 使代入的Gamma_x = gamma_state_adaption_ * gamma_ratio_state_

  UpdateAdaption(&gain_state_adaption_, state_action_,
                 gamma_state_adaption_ * gamma_ratio_state_);

  UpdateAdaption(&gain_input_adaption_, input_desired_,
                 gamma_input_adaption_ * gamma_ratio_input_);

  // 一阶时，theta_x是标量
  // theta_x>0 或 theta_x<限值量 或 theta_r>限值量 或 theta_r<1, 这怎么得到的？

  // revert the adaption law updates if they are beyond the clamping values
  if (model_order_ == 1 && adaption_clamping_enabled &&
      (gain_state_adaption_(0, 0) > 0.0 ||
       gain_state_adaption_(0, 0) < gain_state_clamping_(0, 0) ||
       gain_input_adaption_(0, 0) > gain_input_clamping_(0, 0) ||
       gain_input_adaption_(0, 0) < 1.0)) {
    gain_state_adaption_.col(0) = gain_state_adaption_.col(1);
    gain_input_adaption_.col(0) = gain_input_adaption_.col(1);
  }

  // update the generated control based on the adaptive law
  // u(k) = theta_x^T(k) * x(k) + theta_r^T(k) * r(k)
  double control_unbounded =
      gain_state_adaption_.col(0).transpose() * state_action_.col(0) +
      gain_input_adaption_(0, 0) * input_desired_(0, 0);

  // mrac输出的控制量（转角，转角速度）的限幅滤波
  double control = 0.0;
  saturation_status_control_ =
      BoundOutput(control_unbounded, control_previous_, &control);

  // update the anti-windup compensation if applied
  // 控制器中使用积分模块，必然会出现积分饱和吗？对微分的数值积分也会出现积分饱和吗？
  AntiWindupCompensation(control_unbounded, control_previous_);

  // update the previous value for next iteration
  gain_state_adaption_.col(1) = gain_state_adaption_.col(0);
  gain_input_adaption_.col(1) = gain_input_adaption_.col(0);
  state_reference_.col(1) = state_reference_.col(0);
  state_action_.col(1) = state_action_.col(0);
  input_desired_.col(1) = input_desired_.col(0);
  control_previous_ = control;
  return control;
}

void MracController::Reset() {
  // reset the overall states
  ResetStates();
  // reset the adaptive gains
  ResetGains();
  // reset all the externally-setting, non-conf control parameters
  gamma_ratio_state_ = 1.0;
  gamma_ratio_input_ = 1.0;
  gamma_ratio_nonlinear_ = 1.0;
}

void MracController::ResetStates() {
  // reset the inputs and outputs of the closed-loop MRAC controller
  control_previous_ = 0.0;
  input_desired_.setZero(1, 2);
  // reset the internal states, anti-windup compensations and status
  state_action_.setZero(model_order_, 2);
  state_reference_.setZero(model_order_, 2);
  compensation_anti_windup_.setZero(model_order_, 2);
  saturation_status_reference_ = 0;
  saturation_status_control_ = 0;
}

void MracController::ResetGains() {
  gain_state_adaption_.setZero(model_order_, 2);
  gain_input_adaption_ = Matrix::Ones(1, 2);
  gain_nonlinear_adaption_.setZero(1, 2);
  gain_state_adaption_.col(1) = gain_state_adaption_init_;
  gain_input_adaption_.col(1) = gain_input_adaption_init_;
}

void MracController::Init(const MracConf &mrac_conf,
                          const LatencyParam &latency_param, const double dt) {
  control_previous_ = 0.0;
  saturation_status_control_ = 0;
  saturation_status_reference_ = 0;
  ts_ = dt;
  // Initialize the common model parameters
  model_order_ = mrac_conf.mrac_model_order();
  // Initialize the saturation limits
  // bound_ratio_在物理极限的基础上再做限幅，默认值1，可给小于1的数
  bound_ratio_ = mrac_conf.mrac_saturation_level();
  // Initialize the system states
  input_desired_ = Matrix::Zero(1, 2);  // r
  state_action_ = Matrix::Zero(model_order_, 2);  // xp
  state_reference_ = Matrix::Zero(model_order_, 2);  // xm
  // Initialize the adaptive control gains
  gain_state_adaption_ = Matrix::Zero(model_order_, 2);  // theta_x
  gain_input_adaption_ = Matrix::Ones(1, 2);  // theta_r
  gain_nonlinear_adaption_ = Matrix::Zero(1, 2); //
  // 基于物理意义，对时间常数tau限幅，利用tau计算以下量的限幅
  gain_state_clamping_ = Matrix::Zero(model_order_, 1);  // theta_x_clamping 
  gain_input_clamping_ = Matrix::Ones(1, 1);  // theta_r_clamping 
  gain_nonlinear_clamping_ = Matrix::Zero(1, 1);
  // 一般做法，初始值置0，代码中选择mrac最优控制的对应自适应律的值
  gain_state_adaption_init_ = Matrix::Zero(model_order_, 1);  // theta_X0
  gain_input_adaption_init_ = Matrix::Ones(1, 1);  // theta_r0
  gain_nonlinear_adaption_init_ = Matrix::Zero(1, 1);
  // Initialize the adaptive convergence gains and anti-windup gains
  // 代码中使用它时，给它乘了一个系数
  gamma_state_adaption_ = Matrix::Zero(model_order_, model_order_);  // Gamma_x
  gamma_input_adaption_ = Matrix::Zero(1, 1);  // Gamma_r
  gamma_nonlinear_adaption_ = Matrix::Zero(1, 1);
  // 抗积分饱和环节要乘的系数，为0时没有抗积分饱和
  gain_anti_windup_ = Matrix::Zero(model_order_, model_order_);
  compensation_anti_windup_ = Matrix::Zero(model_order_, 2);
  // Initialize the reference model parameters
  matrix_a_reference_ = Matrix::Zero(model_order_, model_order_);  // Am
  matrix_b_reference_ = Matrix::Zero(model_order_, 1);  // bm
  // SetAdaptionModel设置参考模型wn,tau等参数
  // BuildReferenceModel利用wn，tau计算参考模型的Am，bm矩阵
  // 这里还把这两过程的状态返回值取交集
  reference_model_enabled_ =
      (SetReferenceModel(mrac_conf).ok() && BuildReferenceModel().ok());
  // Initialize the adaption model parameters
  matrix_p_adaption_ = Matrix::Zero(model_order_, model_order_);
  // 实际系统的b矩阵
  matrix_b_adaption_ = Matrix::Zero(model_order_, 1);  // bp
  // 根据系统的阶数，给Gamma_x, 抗积分饱和增益系数， P矩阵， Gamma_r, Gamma_nonlinear 赋值
  adaption_model_enabled_ =
      (SetAdaptionModel(mrac_conf).ok() && BuildAdaptionModel().ok());
  // 利用实际系统的一阶阶跃响应或者二阶阶跃响应来估计实际系统的A和B矩阵
  EstimateInitialGains(latency_param);
  gain_state_adaption_.col(1) = gain_state_adaption_init_;
  gain_input_adaption_.col(1) = gain_input_adaption_init_;
}

// 代码默认读取的是lat_controller_conf.pb.txt
// 其中reference_time_constant存在
// reference_natural_frequency存在
// reference_damping_ratio存在
// clamping_time_constant不存在，因此tau_clamping_不存在
// 因此adaption_clamping_enabled默认是false的，即没有tau的限幅存在

Status MracController::SetReferenceModel(const MracConf &mrac_conf) {
  const double Epsilon = 0.000001;
  if (((mrac_conf.reference_time_constant() < Epsilon && model_order_ == 1)) ||
      ((mrac_conf.reference_natural_frequency() < Epsilon &&
        model_order_ == 2))) {
    const auto error_msg = absl::StrCat(
        "mrac controller error: reference model time-constant parameter: ",
        mrac_conf.reference_time_constant(),
        "and natural frequency parameter: ",
        mrac_conf.reference_natural_frequency(),
        " in configuration file are not reasonable with respect to the "
        "reference model order: ",
        model_order_);
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_INIT_ERROR, error_msg);
  }
  tau_reference_ = mrac_conf.reference_time_constant();
  wn_reference_ = mrac_conf.reference_natural_frequency();
  zeta_reference_ = mrac_conf.reference_damping_ratio();
  tau_clamping_ = mrac_conf.clamping_time_constant();
  adaption_clamping_enabled = (tau_clamping_ >= Epsilon);
  return Status::OK();
}

// 根据系统的阶数，给Gamma_x, 抗积分饱和增益系数， P矩阵， Gamma_r, Gamma_nonlinear 赋值

Status MracController::SetAdaptionModel(const MracConf &mrac_conf) {
  const int p_size = mrac_conf.adaption_matrix_p_size();
  const int x_size = mrac_conf.adaption_state_gain_size();
  const int aw_size = mrac_conf.anti_windup_compensation_gain_size();
  if (p_size != model_order_ * model_order_ || x_size > model_order_ ||
      aw_size > model_order_) {
    const auto error_msg = absl::StrCat(
        "mrac controller error: adaption matrix p element number: ", p_size,
        ", state gain number: ", x_size,
        ", and anti-windup compensation gain number: ", aw_size,
        " in configuration file do not match the model number: ", model_order_);
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_INIT_ERROR, error_msg);
  }
  for (int i = 0; i < model_order_; ++i) {
    gamma_state_adaption_(i, i) =
        (i <= x_size - 1) ? mrac_conf.adaption_state_gain(i)
                          : mrac_conf.adaption_state_gain(x_size - 1);
    gain_anti_windup_(i, i) =
        (i <= aw_size - 1)
            ? mrac_conf.anti_windup_compensation_gain(i)
            : mrac_conf.anti_windup_compensation_gain(aw_size - 1);
    for (int j = 0; j < model_order_; ++j) {
      matrix_p_adaption_(i, j) =
          mrac_conf.adaption_matrix_p(i * model_order_ + j);
    }
  }
  gamma_input_adaption_(0, 0) = mrac_conf.adaption_desired_gain();
  gamma_nonlinear_adaption_(0, 0) = mrac_conf.adaption_nonlinear_gain();
  return Status::OK();
}

// 利用tau，wn，zeta计算参考模型的矩阵Am和Bm

Status MracController::BuildReferenceModel() {
  if (model_order_ > 2) {
    const auto error_msg =
        absl::StrCat("mrac controller error: reference model order ",
                     model_order_, " is beyond the designed range");
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_INIT_ERROR, error_msg);
  }
  if (model_order_ == 1) {
    matrix_a_reference_(0, 0) = -1.0 / tau_reference_;
    matrix_b_reference_(0, 0) = 1.0 / tau_reference_;
  } else if (model_order_ == 2) {
    matrix_a_reference_(0, 1) = 1.0;
    matrix_a_reference_(1, 0) = -wn_reference_ * wn_reference_;
    matrix_a_reference_(1, 1) = -2 * zeta_reference_ * wn_reference_;
    matrix_b_reference_(1, 0) = wn_reference_ * wn_reference_;
  }
  return Status::OK();
}

// 构建自适应律的模型，计算自适应律微分方程中的矩阵P和b

Status MracController::BuildAdaptionModel() {
  if (model_order_ > 2) {
    const auto error_msg =
        absl::StrCat("mrac controller error: adaption model order ",
                     model_order_, " is beyond the designed range");
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_INIT_ERROR, error_msg);
  }
  if (model_order_ == 1) {
    // 一阶系统的实际系统b矩阵是1 * b，是一个标量
    matrix_b_adaption_(0, 0) = 1.0;
  } else if (model_order_ == 2) {
    // 定义的实际系统的b矩阵,为什么这里要用参考系统的wn去定义，不应该用wp吗？
    // 二阶系统的实际系统的b矩阵是[0, wn^2]
    matrix_b_adaption_(1, 0) = wn_reference_ * wn_reference_;
  }
  if (!CheckLyapunovPD(matrix_a_reference_, matrix_p_adaption_)) {
    const std::string error_msg =
        "Solution of the algebraic Lyapunov equation is not symmetric positive "
        "definite";
    AERROR << error_msg;
    return Status(ErrorCode::CONTROL_INIT_ERROR, error_msg);
  }
  return Status::OK();
}

// Cholesky 分解是把一个对称正定的矩阵表示成一个下三角矩阵L和其转置的乘积的分解。
// 它要求矩阵的所有特征值必须大于零，故分解的下三角的对角元也是大于零的。
// Cholesky分解法又称平方根法，是当A为实对称正定矩阵时，LU三角分解法的变形。 
// 把Q分界为L，定义为llt_matrix_q

bool MracController::CheckLyapunovPD(const Matrix matrix_a,
                                     const Matrix matrix_p) const {
  Matrix matrix_q = -matrix_p * matrix_a - matrix_a.transpose() * matrix_p;
  Eigen::LLT<Matrix> llt_matrix_q(matrix_q);
  // if matrix Q is not symmetric or the Cholesky decomposition (LLT) failed
  // due to the matrix Q are not positive definite

  return (matrix_q.isApprox(matrix_q.transpose()) &&
          llt_matrix_q.info() != Eigen::NumericalIssue);
}

// 利用一阶和二阶的阶跃响应的动态特性指标来估计实际系统的A和B

void MracController::EstimateInitialGains(const LatencyParam &latency_param) {
  // 上升时间 = 死区时间+上升时间
  const double rise_time_estimate =
      latency_param.dead_time() + latency_param.rise_time();
  // 调节时间 = 死区时间+调节时间
  const double settling_time_estimate =
      latency_param.dead_time() + latency_param.settling_time();
  // A矩阵的估计值，利用该值来建立执行器动力学模型
  Matrix matrix_a_estimate = Matrix::Zero(model_order_, model_order_);
  Matrix matrix_b_estimate = Matrix::Zero(model_order_, 1);
  Matrix matrix_a_clamping = Matrix::Zero(model_order_, model_order_);
  Matrix matrix_b_clamping = Matrix::Zero(model_order_, 1);

  // 参照《自动控制原理》，一阶系统传递函数G(s) = 1/(tau*s + 1)
  // td-延迟时间 tr-上升时间 tp-峰值时间 ts-调节时间
  // 一阶系统的单位阶跃响应：一阶系统的动态性能指标tr=2.2*tau，ts=3*tau
  // tau是一阶系统的惯性，惯性越小，响应越快

  if (model_order_ == 1 &&
      (rise_time_estimate >= ts_ || settling_time_estimate >= ts_)) {
    const double tau_estimate = (rise_time_estimate >= ts_)
                                    ? rise_time_estimate / 2.2
                                    : settling_time_estimate / 4.0;
    // generate the initial adaptive gains

    // Y(s) / U(s) = 1/(tau*s + 1)
    // Y(s)*tau*s + Y(s) = U(s)
    // 拉氏反变换tau * dot_y(t) + y(t) = u(t)
    // dot_y(t) = -1/tau * y(t) + 1/tau *u(t)
    // matrix_a_estimate = -1/tau，是一阶系统的实际系统的a
    // matrix_b_estimate = 1/tau，是一阶系统的实际系统的b

    matrix_a_estimate(0, 0) = -1.0 / tau_estimate;
    matrix_b_estimate(0, 0) = 1.0 / tau_estimate;

    // 理想的常量theta_x_star = (am - a) / b
    // 理想的常量theta_r_star = bm / b
    // 通常初值选（0，0），这里选theta_x和theta_r的解析解的最优值，只是为了加快响应速度

    gain_state_adaption_init_(0, 0) =
        (matrix_a_reference_(0, 0) - matrix_a_estimate(0, 0)) /
        matrix_b_estimate(0, 0);
    gain_input_adaption_init_(0, 0) =
        matrix_b_reference_(0, 0) / matrix_b_estimate(0, 0);
    
    // adaption_clamping_enabled默认是假的
    // tau的边界应该根据物理意义来设定，比如一阶电系统tau=RC
    // 用tau的边界来计算a和b的边界
    // 再用a和b的边界来计算theta_x_star和theta_r_star的边界，公式同上

    // generate the clamping bounds for adaptive gains
    if (adaption_clamping_enabled) {
      matrix_a_clamping(0, 0) = -1.0 / tau_clamping_;
      matrix_b_clamping(0, 0) = 1.0 / tau_clamping_;
      gain_state_clamping_(0, 0) =
          (matrix_a_clamping(0, 0) - matrix_a_estimate(0, 0)) /
          matrix_b_estimate(0, 0);
      gain_input_clamping_(0, 0) =
          matrix_b_clamping(0, 0) / matrix_b_estimate(0, 0);
    }
  } else if (model_order_ == 2 &&
             (rise_time_estimate >= ts_ && settling_time_estimate >= ts_)) {
    // 参照《自动控制原理》，根据无零点欠阻尼二阶系统的单位阶跃响应中指标的公式
    // tr = 1.8 / wn, tr * ts = 4.6 / zeta 出处何在？
    const double wn_estimate = 1.8 / rise_time_estimate;
    const double zeta_estimate =
        4.6 / (rise_time_estimate * settling_time_estimate);
    // matrix_a_estimate，是二阶系统的实际系统的A
    // matrix_b_estimate，是二阶系统的实际系统的B
    // 这里应该用wp来表达实际系统，wn容易引起歧义
    matrix_a_estimate(0, 1) = 1.0;
    matrix_a_estimate(1, 0) = -wn_estimate * wn_estimate;
    matrix_a_estimate(1, 1) = -2 * zeta_estimate * wn_estimate;
    matrix_b_estimate(1, 0) = wn_estimate * wn_estimate;
    // 初始值的取法同一阶系统
    gain_state_adaption_init_.col(0) =
        (common::math::PseudoInverse<double, 2, 1>(matrix_b_estimate) *
         (matrix_a_reference_ - matrix_a_estimate))
            .transpose();
    gain_input_adaption_init_.col(0) =
        (common::math::PseudoInverse<double, 2, 1>(matrix_b_estimate) *
         matrix_b_reference_)
            .transpose();
  } else {
    AWARN << "No pre-known actuation dynamics; the initial states of the "
             "adaptive gains are set as zeros";
  }
}

// 直接在时域内，采用梯形公式计算数值积分，可将连续微分方程离散化为：
// xm(k+1) = (I - A*Ts/2)^-1 * [ (I + A*Ts/2)*Am + bm*Ts/2 * ( r(k+1) + r(k) ) ]
// 公式中的 1/2 * { r(k)和r(k+1) }直接由公式推导而出，并不是网上说的apollo自己设计的用于“取均值”

void MracController::UpdateReference() {
  Matrix matrix_i = Matrix::Identity(model_order_, model_order_);
  state_reference_.col(0) =
      (matrix_i - ts_ * 0.5 * matrix_a_reference_).inverse() *
      ((matrix_i + ts_ * 0.5 * matrix_a_reference_) * state_reference_.col(1) +
       ts_ * 0.5 * matrix_b_reference_ *
           (input_desired_(0, 0) + input_desired_(0, 1)));
}

// 继续采用梯形公式进行数值求解，不过并没有像UpdateReference那样整合项
// theta_x（k+1） - theta_x（k） = -Ts*Gamma_x/2 * [ xp(k+1)e(k+1)^T + xp(k)e(k)^T) ] * p * bp
// 或者 theta_r（k+1） - theta_r（k） = -Ts*Gamma_r/2 * [ r(k+1)e(k+1)^T + r(k)e(k)^T) ] * p * bp

void MracController::UpdateAdaption(Matrix *law_adp, const Matrix state_adp,
                                    const Matrix gain_adp) {
  // e = x - xm
  Matrix state_error = state_action_ - state_reference_;
  law_adp->col(0) =
      law_adp->col(1) -
      0.5 * ts_ * gain_adp *
          (state_adp.col(0) * (state_error.col(0).transpose() +
                               compensation_anti_windup_.col(0).transpose()) +
           state_adp.col(1) * (state_error.col(1).transpose() +
                               compensation_anti_windup_.col(1).transpose())) *
          matrix_p_adaption_ * matrix_b_adaption_;
}

// 抗积分饱和

void MracController::AntiWindupCompensation(const double control_command,
                                            const double previous_command) {
  Matrix offset_windup = Matrix::Zero(model_order_, 1);
  // 记录超出幅值的转角的delta值
  offset_windup(0, 0) =
      ((control_command > bound_command_) ? bound_command_ - control_command
                                          : 0.0) +
      ((control_command < -bound_command_) ? -bound_command_ - control_command
                                           : 0.0);
  // 记录超出幅值的转角速度的delta值
  if (model_order_ > 1) {
    offset_windup(1, 0) =
        ((control_command > previous_command + bound_command_rate_ * ts_)
             ? bound_command_rate_ - (control_command - previous_command) / ts_
             : 0.0) +
        ((control_command < previous_command - bound_command_rate_ * ts_)
             ? -bound_command_rate_ - (control_command - previous_command) / ts_
             : 0.0);
  }
  compensation_anti_windup_.col(1) = compensation_anti_windup_.col(0);
  // gain_anti_windup_默认值是0
  compensation_anti_windup_.col(0) = gain_anti_windup_ * offset_windup;
}

// 输出值和输出值的变化量都不能超过限值，相当于限值了方向盘转角和方向盘转角速度

int MracController::BoundOutput(const double output_unbounded,
                                const double previous_output, double *output) {
  int status = 0;
  if (output_unbounded > bound_command_ ||
      output_unbounded > previous_output + bound_command_rate_ * ts_) {
    *output = (bound_command_ < previous_output + bound_command_rate_ * ts_)
                  ? bound_command_
                  : previous_output + bound_command_rate_ * ts_;
    // if output exceeds the upper bound, then status = 1; while if output
    // changing rate exceeds the upper rate bound, then status = 2
    status =
        (bound_command_ < previous_output + bound_command_rate_ * ts_) ? 1 : 2;
  } else if (output_unbounded < -bound_command_ ||
             output_unbounded < previous_output - bound_command_rate_ * ts_) {
    *output = (-bound_command_ > previous_output - bound_command_rate_ * ts_)
                  ? -bound_command_
                  : previous_output - bound_command_rate_ * ts_;
    // if output exceeds the lower bound, then status = -1; while if output
    // changing rate exceeds the lower rate bound, then status = -2
    status = (-bound_command_ > previous_output - bound_command_rate_ * ts_)
                 ? -1
                 : -2;
  } else {
    *output = output_unbounded;
    // if output does not violate neither bound nor rate bound, then status = 0
    status = 0;
  }
  return status;
}

// 参考模型微分方程的初始值，先把第二列赋值，后边再利用第二列来数值计算更新第一列

void MracController::SetInitialReferenceState(
    const Matrix &state_reference_init) {
  if (state_reference_init.rows() != model_order_ ||
      state_reference_init.cols() != 1) {
    AWARN << "failed to set the initial reference states, due to the given "
             "state size: "
          << state_reference_init.rows() << " x " << state_reference_init.cols()
          << " doesn't match the model order: " << model_order_;
  } else {
    state_reference_.col(1) = state_reference_init;
  }
}

// 实际系统的微分方程的初始值

void MracController::SetInitialActionState(const Matrix &state_action_init) {
  if (state_action_init.rows() != model_order_ ||
      state_action_init.cols() != 1) {
    AWARN << "failed to set the initial action states, due to the given "
             "state size: "
          << state_action_init.rows() << " x " << state_action_init.cols()
          << " doesn't match the model order: " << model_order_;
  } else {
    state_action_.col(1) = state_action_init;
  }
}

// 参考模型微分方程中r的初始值

void MracController::SetInitialCommand(const double command_init) {
  input_desired_(0, 1) = command_init;
}

void MracController::SetInitialStateAdaptionGain(
    const Matrix &gain_state_adaption_init) {
  if (gain_state_adaption_init.rows() != model_order_ ||
      gain_state_adaption_init.cols() != 1) {
    AWARN << "failed to set the initial state adaption gains, due to the given "
             "state size: "
          << gain_state_adaption_init.rows() << " x "
          << gain_state_adaption_init.cols()
          << " doesn't match the model order: " << model_order_;
  } else {
    gain_state_adaption_.col(1) = gain_state_adaption_init;
  }
}

// 自适应律微分方程中theta_r的初始值

void MracController::SetInitialInputAdaptionGain(
    const double gain_input_adaption_init) {
  gain_input_adaption_(0, 1) = gain_input_adaption_init;
}

// 自适应律微分方程中theta_nonlinear的初始值

void MracController::SetInitialNonlinearAdaptionGain(
    const double gain_nonlinear_adaption_init) {
  gain_nonlinear_adaption_(0, 1) = gain_nonlinear_adaption_init;
}

// 自适应律微分方程中Gamma_x的要乘的系数的值
// 这个标量做为系数，它的值一旦赋定，在代码中就不再更改

void MracController::SetStateAdaptionRate(const double ratio_state) {
  if (ratio_state < 0.0) {
    AWARN << "failed to set the state adaption rate, due to new ratio < 0; the "
             "current ratio is still: "
          << gamma_ratio_state_;
  } else {
    gamma_ratio_state_ = ratio_state;
  }
}

// 自适应律微分方程中Gamma_r的要乘的系数的值
// 这个标量做为系数，它的值一旦赋定，在代码中就不再更改

void MracController::SetInputAdaptionRate(const double ratio_input) {
  if (ratio_input < 0.0) {
    AWARN << "failed to set the input adaption rate, due to new ratio < 0; the "
             "current ratio is still: "
          << gamma_ratio_input_;
  } else {
    gamma_ratio_input_ = ratio_input;
  }
}

// 自适应律微分方程中Gamma_nonlinear的要乘的系数的值
// 这个标量做为系数，它的值一旦赋定，在代码中就不再更改

void MracController::SetNonlinearAdaptionRate(const double ratio_nonlinear) {
  if (ratio_nonlinear < 0.0) {
    AWARN << "failed to set the nonlinear adaption rate, due to new ratio < 0; "
             "the current ratio is still: "
          << gamma_ratio_nonlinear_;
  } else {
    gamma_ratio_nonlinear_ = ratio_nonlinear;
  }
}

double MracController::StateAdaptionRate() const { return gamma_ratio_state_; }

double MracController::InputAdaptionRate() const { return gamma_ratio_input_; }

double MracController::NonlinearAdaptionRate() const {
  return gamma_ratio_nonlinear_;
}

// 参考模型中状态量的饱和

int MracController::ReferenceSaturationStatus() const {
  return saturation_status_reference_;
}

// mrac输出量的饱和

int MracController::ControlSaturationStatus() const {
  return saturation_status_control_;
}

Matrix MracController::CurrentReferenceState() const {
  return state_reference_;
}

Matrix MracController::CurrentStateAdaptionGain() const {
  return gain_state_adaption_;
}

Matrix MracController::CurrentInputAdaptionGain() const {
  return gain_input_adaption_;
}

Matrix MracController::CurrentNonlinearAdaptionGain() const {
  return gain_nonlinear_adaption_;
}

}  // namespace control
}  // namespace apollo
