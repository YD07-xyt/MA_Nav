#pragma once

#include "planner/opt/traj_optimizer.hpp"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <OsqpEigen/Solver.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace controller {

class LMpc {
public:
  struct LMpcParam {
    int N = 20;
    double dt = 0.05;
    Eigen::Vector3d u_min;
    Eigen::Vector3d u_max;
    Eigen::Vector3d x_min;
    Eigen::Vector3d x_max;
    Eigen::Matrix3d Q;
    Eigen::Matrix3d R;
  } param_;

public:
  LMpc(LMpcParam lmpc_param) {
    // ---------- 初始化参数 ----------
    param_=lmpc_param;
    // param_.N = 20;
    // param_.dt = 0.1;

    A = Eigen::Matrix3d::Identity();
    B = Eigen::Matrix3d::Identity() * param_.dt;

    // // 权重矩阵
    // param_.Q = Eigen::Matrix3d::Zero();
    // param_.Q.diagonal() << 1.0, 1.0, 0.1; // 位置权重高，角度权重低
    // param_.R = Eigen::Matrix3d::Zero();
    // param_.R.diagonal() << 0.1, 0.1, 0.05; // 控制平滑

    // // MPC 参数
    // // 控制边界需与参考轨迹可达速度一致（traj_opt 中 max_v = 3.0 m/s）
    // param_.u_min = Eigen::Vector3d(-3.0, -3.0, -2.0);
    // param_.u_max = Eigen::Vector3d(3.0, 3.0, 2.0);
    // param_.x_min = -Eigen::Vector3d::Constant(OsqpEigen::INFTY);
    // param_.x_max = Eigen::Vector3d::Constant(OsqpEigen::INFTY);

    // ---------- 预计算矩阵维度 ----------
    const int N = param_.N;
    const int n_vars = 3 * (N + 1) + 3 * N; // 状态变量 + 控制变量
    const int n_constraints = 3 * (N + 1) + n_vars; // 等式约束 + 不等式约束

    // ---------- 填充 Hessian 矩阵（固定） ----------
    hessian_.resize(n_vars, n_vars);
    hessian_.reserve(n_vars);

    // 状态权重 Q（对应 x_0 ~ x_N）
    for (int i = 0; i < N + 1; ++i) {
      for (int j = 0; j < 3; ++j) {
        hessian_.insert(i * 3 + j, i * 3 + j) = param_.Q.diagonal()[j];
      }
    }
    // 控制权重 R（对应 u_0 ~ u_{N-1}）
    for (int i = 0; i < N; ++i) {
      int offset = 3 * (N + 1) + i * 3;
      for (int j = 0; j < 3; ++j) {
        hessian_.insert(offset + j, offset + j) = param_.R.diagonal()[j];
      }
    }
    hessian_.makeCompressed();

    // ---------- 填充线性约束矩阵（固定） ----------
    linear_matrix_.resize(n_constraints, n_vars);
    linear_matrix_.reserve(2 * n_vars + 3 * N * 3); // 粗略估计

    // ---- 上半部分：等式约束（动力学） ----
    // 第一行：-x0 = -x0 （系数 -I）
    for (int j = 0; j < 3; ++j) {
      linear_matrix_.insert(j, j) = -1.0;
    }
    // 后续行：-x_{k+1} + A*x_k + B*u_k = 0
    for (int k = 0; k < N; ++k) {
      // x_k 系数（A 对角）
      for (int j = 0; j < 3; ++j) {
        linear_matrix_.insert((k + 1) * 3 + j, k * 3 + j) = A(j, j);
      }
      // x_{k+1} 系数（-I）
      for (int j = 0; j < 3; ++j) {
        linear_matrix_.insert((k + 1) * 3 + j, (k + 1) * 3 + j) = -1.0;
      }
      // u_k 系数（B = dt·R(theta0)，机体速度经航向旋转到世界系；
      //           theta0 在求解时按当前航向更新，此处仅建立稀疏模式）
      int u_offset = 3 * (N + 1) + k * 3;
      linear_matrix_.insert((k + 1) * 3 + 0, u_offset + 0) = B(0, 0);
      linear_matrix_.insert((k + 1) * 3 + 0, u_offset + 1) = 0.0;
      linear_matrix_.insert((k + 1) * 3 + 1, u_offset + 0) = 0.0;
      linear_matrix_.insert((k + 1) * 3 + 1, u_offset + 1) = B(1, 1);
      linear_matrix_.insert((k + 1) * 3 + 2, u_offset + 2) = B(2, 2);
    }

    // ---- 下半部分：不等式约束（边界） ----
    int eq_rows = 3 * (N + 1);
    for (int i = 0; i < n_vars; ++i) {
      linear_matrix_.insert(eq_rows + i, i) = 1.0;
    }
    linear_matrix_.makeCompressed();

    // ---------- 初始化 OSQP 求解器（只一次） ----------
    // 先构建零向量（避免表达式模板问题）
    Eigen::VectorXd zero_grad = Eigen::VectorXd::Zero(n_vars);
    Eigen::VectorXd zero_lb = Eigen::VectorXd::Zero(n_constraints);
    Eigen::VectorXd zero_ub = Eigen::VectorXd::Zero(n_constraints);

    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    solver_.data()->setNumberOfVariables(n_vars);
    solver_.data()->setNumberOfConstraints(n_constraints);
    solver_.data()->setHessianMatrix(hessian_);
    solver_.data()->setLinearConstraintsMatrix(linear_matrix_);
    solver_.data()->setGradient(zero_grad);
    solver_.data()->setLowerBound(zero_lb);
    solver_.data()->setUpperBound(zero_ub);

    solver_.initSolver(); 
  }

  // 更新当前机器人位姿
  auto update_current_pose(Eigen::Vector3d current_pose) {
    current_pose_ = current_pose;
  }

  // 核心求解函数：输入样条轨迹，返回预测状态序列
  auto slover(const SplineTrajectory::PPolyND<2, 6> &trajectory)
      -> std::vector<Eigen::Vector3d> {
    // 1. 获取当前状态
    Eigen::Vector3d x0 = current_pose_;
    const int N = param_.N;
    const double dt = param_.dt;
    const double total_duration = trajectory.getDuration();
    // 参考游标跟随机器人实际进度（最近点），而非墙钟时间
    double t_now = update_track_time(x0, trajectory);

    // 2. 生成未来 N+1 步的参考轨迹
    Eigen::Matrix3Xd x_ref(3, N + 1);
    const double theta0 = x0.z();
    const double c0 = std::cos(theta0), s0 = std::sin(theta0);
    double theta_ref_prev = theta0;
    for (int k = 0; k <= N; ++k) {
      double t_eval = t_now + k * dt;
      if (t_eval > total_duration)
        t_eval = total_duration;

      Eigen::Vector2d pos = trajectory.evaluate(t_eval, 0);
      Eigen::Vector2d vel = trajectory.evaluate(t_eval, 1);
      double theta_raw = std::atan2(vel.y(), vel.x());
      // 沿 k 连续解卷绕参考航向（相对当前航向起步），
      // 避免 atan2 在 ±π 边界跳变导致角度代价突增、转向错误
      double dtheta = theta_raw - theta_ref_prev;
      dtheta = std::atan2(std::sin(dtheta), std::cos(dtheta)); // wrap to [-π, π]
      double theta_ref = theta_ref_prev + dtheta;
      theta_ref_prev = theta_ref;

      x_ref.col(k) << pos.x(), pos.y(), theta_ref;
    }

    // 3. 更新梯度向量（时变）
    const int n_vars = 3 * (N + 1) + 3 * N;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n_vars);
    for (int i = 0; i < N + 1; ++i) {
      Eigen::Vector3d Qx_ref = param_.Q * (-x_ref.col(i));
      gradient.segment(i * 3, 3) = Qx_ref;
    }
    // 控制量参考：追踪参考速度（世界系 → 机体系）与参考角速度，
    // 避免只追踪位置导致弯道切弯/偏离轨迹
    for (int k = 0; k < N; ++k) {
      double t_eval = t_now + k * dt;
      if (t_eval > total_duration)
        t_eval = total_duration;
      Eigen::Vector2d vel_w = trajectory.evaluate(t_eval, 1);
      // R(θ0)^T · v_world → body frame
      Eigen::Vector2d vel_b(c0 * vel_w.x() + s0 * vel_w.y(),
                            -s0 * vel_w.x() + c0 * vel_w.y());
      gradient.segment(3 * (N + 1) + k * 3, 2) =
          -param_.R.diagonal().head<2>().cwiseProduct(vel_b);
      // 角速度参考：参考航向的差分
      double w_ref = (x_ref.col(k + 1)(2) - x_ref.col(k)(2)) / dt;
      gradient[3 * (N + 1) + k * 3 + 2] = -param_.R.diagonal()(2) * w_ref;
    }

    // 4. 更新约束边界（时变）
    const int eq_rows = 3 * (N + 1);
    const int n_constraints = eq_rows + n_vars;
    Eigen::VectorXd lower_bound = Eigen::VectorXd::Zero(n_constraints);
    Eigen::VectorXd upper_bound = Eigen::VectorXd::Zero(n_constraints);

    // 等式边界：第一块 -x0
    lower_bound.segment(0, 3) = -x0;
    upper_bound.segment(0, 3) = -x0;
    // 其余等式边界为 0
    lower_bound.segment(3, eq_rows - 3).setZero();
    upper_bound.segment(3, eq_rows - 3).setZero();

    // 不等式边界
    for (int i = 0; i < N + 1; ++i) {
      lower_bound.segment(eq_rows + i * 3, 3) = param_.x_min;
      upper_bound.segment(eq_rows + i * 3, 3) = param_.x_max;
    }
    int offset_ineq = eq_rows + 3 * (N + 1);
    for (int i = 0; i < N; ++i) {
      lower_bound.segment(offset_ineq + i * 3, 3) = param_.u_min;
      upper_bound.segment(offset_ineq + i * 3, 3) = param_.u_max;
    }

    // 4.5 重新构建动力学约束矩阵：
    //     控制 (vx, vy) 是机体坐标系速度，须经当前航向 theta0 旋转到世界系
    //     x_{k+1} = x_k + dt·R(theta0)·[vx; vy],  theta_{k+1} = theta_k + dt·w
    {
      const double theta0 = x0.z();
      const double c = std::cos(theta0);
      const double s = std::sin(theta0);
      const int eq_rows = 3 * (N + 1);
      const int n_constraints = eq_rows + n_vars;
      Eigen::SparseMatrix<double> lin(n_constraints, n_vars);
      lin.reserve(400);

      // 第一行：-x0 = -x0
      for (int j = 0; j < 3; ++j)
        lin.insert(j, j) = -1.0;

      // 动力学等式：-x_{k+1} + A*x_k + B(θ0)*u_k = 0
      for (int k = 0; k < N; ++k) {
        for (int j = 0; j < 3; ++j)
          lin.insert((k + 1) * 3 + j, k * 3 + j) = A(j, j);
        for (int j = 0; j < 3; ++j)
          lin.insert((k + 1) * 3 + j, (k + 1) * 3 + j) = -1.0;
        int uo = 3 * (N + 1) + k * 3;
        lin.insert((k + 1) * 3 + 0, uo + 0) = dt * c;
        lin.insert((k + 1) * 3 + 0, uo + 1) = -dt * s;
        lin.insert((k + 1) * 3 + 1, uo + 0) = dt * s;
        lin.insert((k + 1) * 3 + 1, uo + 1) = dt * c;
        lin.insert((k + 1) * 3 + 2, uo + 2) = dt;
      }

      // 不等式边界（单位矩阵）
      for (int i = 0; i < n_vars; ++i)
        lin.insert(eq_rows + i, i) = 1.0;
      lin.makeCompressed();
      linear_matrix_ = lin;
    }
    solver_.updateLinearConstraintsMatrix(linear_matrix_);

    // 5. 更新求解器（不要调用 initSolver！）
    solver_.updateGradient(gradient);
    solver_.updateBounds(lower_bound, upper_bound);

    // 6. 求解 QP
    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
      return std::vector<Eigen::Vector3d>();
    }

    // 7. 提取结果
    Eigen::VectorXd solution = solver_.getSolution();
    std::vector<Eigen::Vector3d> predicted_states;
    predicted_states.reserve(N + 1);
    for (int i = 0; i <= N; ++i) {
      predicted_states.push_back(solution.segment(i * 3, 3));
    }
    u_k = solution.segment(3 * (N + 1), 3); // 当前控制指令
    return predicted_states;
  }

  auto get_dt_control() const -> float { return param_.dt; }

  // 新轨迹下发时调用，重置沿轨迹的跟踪游标
  void reset_track() { t_track_ = 0.0; }

  /**
   * @brief 新轨迹下发时,把跟踪游标初始化到轨迹上离机器人当前位置最近的点(全轨迹搜索)。
   *
   * 不能用 reset_track() 清零:update_track_time 的搜索窗口只有 [t_track_-0.3, t_track_+3.0],
   * 游标为 0 时只在前 3 秒内找参考,机器人实际进度若已到 t=5s,参考会被拉回起点附近,
   * MPC 看到参考倒退会急刹减速。全轨迹定位后,MPC 参考速度与新轨迹在该点的速度一致
   * (新轨迹起点速度边界 = 机器人实际速度),切换连续不减速。
   *
   * @param pose       当前机器人状态 (x, y, yaw)
   * @param trajectory 新下发的轨迹样条
   * @return 初始游标时间(秒)
   */
  double initialize_track(const Eigen::Vector3d &pose,
                          const SplineTrajectory::PPolyND<2, 6> &trajectory) {
    const double duration = trajectory.getDuration();
    if (duration <= 0.0) {
      t_track_ = 0.0;
      return 0.0;
    }
    double best_t = 0.0;
    double best_d = std::numeric_limits<double>::max();
    const double step = 0.1; // 粗搜即可,后续 update_track_time 会精细跟踪
    for (double t = 0.0; t <= duration + 1e-9; t += step) {
      Eigen::Vector2d p = trajectory.evaluate(t, 0);
      const double d = (p - pose.head<2>()).squaredNorm();
      if (d < best_d) {
        best_d = d;
        best_t = t;
      }
    }
    t_track_ = best_t;
    return t_track_;
  }

public:                // 外部可访问成员（方便直接读取）
  Eigen::Vector3d u_k; // 当前控制指令 (vx, vy, wz)
  double t_now = 0.0; // 当前轨迹时间（由外部循环更新，已不再用于参考采样）

private:
  // 沿轨迹的跟踪游标：取机器人当前位姿在轨迹上的最近点时间，
  // 使参考采样与机器人实际进度绑定，而非跟随墙钟时间（避免落后时反复切角累积偏差）
  double t_track_ = 0.0;

  // 在 t_track_ 附近窗口内搜索轨迹上离 current_pose
  // 最近的点，更新并返回游标时间
  double update_track_time(const Eigen::Vector3d &pose,
                           const SplineTrajectory::PPolyND<2, 6> &trajectory) {
    const double duration = trajectory.getDuration();
    if (duration <= 0.0) {
      t_track_ = 0.0;
      return 0.0;
    }
    // 兜底：若游标越界（如新轨迹尚未 reset_track），
    // 从轨迹中段重新开始搜索，避免搜索窗口为空导致参考卡在轨迹末端
    if (t_track_ > duration) {
      t_track_ = duration * 0.5;
    }
    const double step = 0.05;
    const double t_lo = std::max(0.0, t_track_ - 0.3);
    const double t_hi = std::min(duration, t_track_ + 3.0);
    double best_t = t_track_;
    double best_d = std::numeric_limits<double>::max();
    for (double t = t_lo; t <= t_hi + 1e-9; t += step) {
      Eigen::Vector2d p = trajectory.evaluate(t, 0);
      double d = (p - pose.head<2>()).squaredNorm();
      if (d < best_d) {
        best_d = d;
        best_t = t;
      }
    }
    t_track_ = best_t;
    return t_track_;
  }

private:
  // 系统矩阵
  // double dt;
  Eigen::Matrix3d A;
  Eigen::Matrix3d B;

  // 当前状态
  Eigen::Vector3d current_pose_;

  // OSQP 求解器及固定矩阵
  OsqpEigen::Solver solver_;
  Eigen::SparseMatrix<double> hessian_;
  Eigen::SparseMatrix<double> linear_matrix_;
};

} // namespace controller