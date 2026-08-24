#include "planner/controller/mpc.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace control {

// ============================================================
// 构造函数
// ============================================================
Mpc::Mpc(const Param& param) : param_(param) {
    // 1. 离散化世界系线性模型
    discretize();

    // 2. 初始化 OSQP，并一次性构建常数矩阵 H_ / A_lin_
    init_solver();
}

// ============================================================
// 设置轨迹接口
// ============================================================
void Mpc::set_trajectory(TrajectoryPtr traj) {
    traj_ = std::move(traj);
}

// ============================================================
// 离散化模型
//
// 状态：
//   x = [x, y, yaw, vx, vy, wz]^T
//
// 控制：
//   u = [ax, ay, az]^T
//
// 连续模型：
//   dx/dt = A x + B u
//
// 离散模型：
//   x_{k+1} = A_d x_k + B_d u_k
// ============================================================
void Mpc::discretize() {
    const double dt = param_.dt;

    // A_d = I + A * dt
    A_ = Eigen::Matrix<double, 6, 6>::Identity();
    A_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

    // B_d = B * dt
    B_ = Eigen::Matrix<double, 6, 3>::Zero();
    B_.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * dt;
}

// ============================================================
// 初始化 OSQP
//
// 重点优化：
//   H_ 和 A_lin_ 是常数矩阵，只在初始化时构建一次。
//   每帧只需要更新 g_、lb_、ub_，大幅提升求解频率。
// ============================================================
void Mpc::init_solver() {
    const int N = param_.N;
    const int nx = 6;
    const int nu = 3;

    // 决策变量：
    //   z = [x_0, x_1, ..., x_N, u_0, u_1, ..., u_{N-1}]
    n_vars_ = nx * (N + 1) + nu * N;

    // 约束数量：
    //   1. 初始状态等式：nx
    //   2. 动力学等式：N * nx
    //   3. 状态不等式：(N+1) * nx
    //   4. 控制不等式：N * nu
    const int n_eq = nx + N * nx;
    const int n_ineq_state = (N + 1) * nx;
    const int n_ineq_u = N * nu;
    n_constraints_ = n_eq + n_ineq_state + n_ineq_u;

    H_.resize(n_vars_, n_vars_);
    g_.resize(n_vars_);
    A_lin_.resize(n_constraints_, n_vars_);
    lb_.resize(n_constraints_);
    ub_.resize(n_constraints_);

    // ============================================================
    // 构建 Hessian H_（常数）
    //
    // 代价：
    //   J = Σ (x - x_ref)^T Q (x - x_ref)
    //     + Σ (u - u_ref)^T R (u - u_ref)
    //     + Σ Δu^T Rd Δu
    //
    // OSQP 形式：
    //   min 0.5 * z^T H z + g^T z
    //
    // 所以 H 中：
    //   状态块 = 2 * Q
    //   控制块 = 2 * R
    //   Rd 产生相邻控制量之间的交叉项
    // ============================================================
    H_.setZero();
    std::vector<Eigen::Triplet<double>> h_triplets;

    // 状态代价
    for (int k = 0; k <= N; ++k) {
        const auto& Qk = (k == N) ? param_.QN : param_.Q;
        const int offset = k * nx;

        for (int i = 0; i < nx; ++i) {
            h_triplets.emplace_back(
                offset + i,
                offset + i,
                2.0 * Qk(i, i));
        }
    }

    // 控制代价 + 控制增量代价
    for (int k = 0; k < N; ++k) {
        const int offset = nx * (N + 1) + k * nu;

        for (int i = 0; i < nu; ++i) {
            h_triplets.emplace_back(
                offset + i,
                offset + i,
                2.0 * param_.R(i, i));
        }

        // Rd：惩罚 u_k - u_{k-1}
        if (k > 0) {
            const int prev_offset = nx * (N + 1) + (k - 1) * nu;

            for (int i = 0; i < nu; ++i) {
                // u_k 对角
                h_triplets.emplace_back(
                    offset + i,
                    offset + i,
                    2.0 * param_.Rd(i, i));

                // u_{k-1} 对角
                h_triplets.emplace_back(
                    prev_offset + i,
                    prev_offset + i,
                    2.0 * param_.Rd(i, i));

                // 交叉项
                h_triplets.emplace_back(
                    offset + i,
                    prev_offset + i,
                    -2.0 * param_.Rd(i, i));

                h_triplets.emplace_back(
                    prev_offset + i,
                    offset + i,
                    -2.0 * param_.Rd(i, i));
            }
        }
    }

    H_.setFromTriplets(h_triplets.begin(), h_triplets.end());

    // ============================================================
    // 构建线性约束矩阵 A_lin_（常数）
    //
    // 约束顺序：
    //   1. 初始状态等式：x_0 = x_current
    //   2. 动力学等式：x_{k+1} - A_d x_k - B_d u_k = 0
    //   3. 状态不等式：x_min <= x_k <= x_max
    //   4. 控制不等式：u_min <= u_k <= u_max
    // ============================================================
    A_lin_.setZero();
    std::vector<Eigen::Triplet<double>> a_triplets;
    int row = 0;

    // 4.1 初始状态等式约束
    for (int i = 0; i < nx; ++i) {
        a_triplets.emplace_back(row, i, 1.0);
        ++row;
    }

    // 4.2 动力学等式约束
    for (int k = 0; k < N; ++k) {
        const int xk_offset = k * nx;
        const int xk1_offset = (k + 1) * nx;
        const int uk_offset = nx * (N + 1) + k * nu;

        // x_{k+1}
        for (int i = 0; i < nx; ++i) {
            a_triplets.emplace_back(row, xk1_offset + i, 1.0);
        }

        // -A_d x_k
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nx; ++j) {
                if (std::abs(A_(i, j)) > 1e-12) {
                    a_triplets.emplace_back(
                        row,
                        xk_offset + j,
                        -A_(i, j));
                }
            }
        }

        // -B_d u_k
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nu; ++j) {
                if (std::abs(B_(i, j)) > 1e-12) {
                    a_triplets.emplace_back(
                        row,
                        uk_offset + j,
                        -B_(i, j));
                }
            }
        }

        ++row;
    }

    // 4.3 状态不等式约束
    for (int k = 0; k <= N; ++k) {
        const int offset = k * nx;

        for (int i = 0; i < nx; ++i) {
            a_triplets.emplace_back(row, offset + i, 1.0);
            ++row;
        }
    }

    // 4.4 控制不等式约束
    for (int k = 0; k < N; ++k) {
        const int offset = nx * (N + 1) + k * nu;

        for (int i = 0; i < nu; ++i) {
            a_triplets.emplace_back(row, offset + i, 1.0);
            ++row;
        }
    }

    A_lin_.setFromTriplets(a_triplets.begin(), a_triplets.end());

    // ============================================================
    // 初始化 OSQP
    // ============================================================
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);

    solver_.data()->setNumberOfVariables(n_vars_);
    solver_.data()->setNumberOfConstraints(n_constraints_);

    solver_.data()->setHessianMatrix(H_);
    solver_.data()->setGradient(g_);
    solver_.data()->setLinearConstraintsMatrix(A_lin_);
    solver_.data()->setLowerBound(lb_);
    solver_.data()->setUpperBound(ub_);

    solver_.initSolver();
}

// ============================================================
// 构建每帧 QP 数据
//
// 优化：
//   H_ 和 A_lin_ 已经固定，这里只更新：
//     - g_：梯度
//     - lb_ / ub_：约束边界
// ============================================================
bool Mpc::build_problem(
    const StateVector& x0,
    double t_now) {

    if (!traj_ || !traj_->valid()) {
        return false;
    }

    const int N = param_.N;
    const int nx = 6;
    const int nu = 3;

    // 1. 采样参考轨迹
    std::vector<TarjectoryInterfaces::ReferencePoint> refs;
    traj_->sample_sequence(t_now, param_.dt, N, refs);

    if (refs.size() < static_cast<size_t>(N + 1)) {
        return false;
    }

    // ============================================================
    // 2. 更新梯度 g_
    //
    // 代价展开：
    //   (x - x_ref)^T Q (x - x_ref)
    // = x^T Q x - 2 * x_ref^T Q x + const
    //
    // 所以对 x 的线性项是：
    //   -2 * Q * x_ref
    //
    // 控制部分同理：
    //   -2 * R * u_ref
    // ============================================================
    g_.setZero();

    for (int k = 0; k <= N; ++k) {
        const auto& Qk = (k == N) ? param_.QN : param_.Q;
        const int offset = k * nx;

        g_.segment(offset, nx) = -2.0 * Qk * refs[k].state;
    }

    for (int k = 0; k < N; ++k) {
        const int offset = nx * (N + 1) + k * nu;

        g_.segment(offset, nu) = -2.0 * param_.R * refs[k].input;
    }

    // ============================================================
    // 3. 更新约束边界 lb_ / ub_
    // ============================================================
    lb_.setZero();
    ub_.setZero();

    int row = 0;

    // 3.1 初始状态等式约束
    for (int i = 0; i < nx; ++i) {
        lb_(row) = x0(i);
        ub_(row) = x0(i);
        ++row;
    }

    // 3.2 动力学等式约束
    for (int k = 0; k < N; ++k) {
        lb_(row) = 0.0;
        ub_(row) = 0.0;
        ++row;
    }

    // 3.3 状态不等式约束
    for (int k = 0; k <= N; ++k) {
        for (int i = 0; i < nx; ++i) {
            lb_(row) = param_.x_min(i);
            ub_(row) = param_.x_max(i);
            ++row;
        }
    }

    // 3.4 控制不等式约束
    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < nu; ++i) {
            lb_(row) = param_.u_min(i);
            ub_(row) = param_.u_max(i);
            ++row;
        }
    }

    return true;
}

// ============================================================
// 求解 MPC，只返回当前控制量
// ============================================================
bool Mpc::solve(
    const StateVector& x0,
    double t_now,
    InputVector& u_cmd) {

    std::vector<StateVector> states;
    std::vector<InputVector> inputs;

    return solve(x0, t_now, u_cmd, states, inputs);
}

// ============================================================
// 求解 MPC，返回控制量和预测轨迹
// ============================================================
bool Mpc::solve(
    const StateVector& x0,
    double t_now,
    InputVector& u_cmd,
    std::vector<StateVector>& predicted_states,
    std::vector<InputVector>& predicted_inputs) {

    // 1. 构建 QP 数据
    if (!build_problem(x0, t_now)) {
        return false;
    }

    // 2. 更新 OSQP
    //
    // H_ 和 A_lin_ 不变，不需要 update
    solver_.updateGradient(g_);
    solver_.updateBounds(lb_, ub_);

    // 3. 求解
    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        return false;
    }

    // 4. 提取解
    Eigen::VectorXd solution = solver_.getSolution();

    const int N = param_.N;
    const int nx = 6;
    const int nu = 3;

    // 4.1 当前控制量
    u_cmd = solution.segment<3>(nx * (N + 1));

    // 4.2 预测状态和控制序列
    predicted_states.clear();
    predicted_inputs.clear();

    for (int k = 0; k <= N; ++k) {
        predicted_states.push_back(solution.segment<6>(k * nx));
    }

    for (int k = 0; k < N; ++k) {
        predicted_inputs.push_back(
            solution.segment<3>(nx * (N + 1) + k * nu));
    }

    // 5. 保存上一次控制量，后续可用于 Rd
    param_.u_prev = u_cmd;

    return true;
}

}  // namespace control