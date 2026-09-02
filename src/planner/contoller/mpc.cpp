#include "planner/controller/mpc.h"
#include "utils/logger.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace control {

// ============================================================
// 构造函数
// ============================================================
Mpc::Mpc(const Param& param): param_(param) {
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
//   x = [x, y, vx, vy]^T
//
// 控制：
//   u = [ax, ay]^T
//
// 离散模型：
//   x_{k+1} = A_d x_k + B_d u_k
// ============================================================
void Mpc::discretize() {
    const double dt = param_.dt;
    constexpr int kStateDim = K_STATE_DIM; // 4
    constexpr int kInputDim = K_INPUT_DIM; // 2
    constexpr int kVelDim = kStateDim - 2; // 2

    // A_d = I + A * dt
    A_ = Eigen::Matrix<double, kStateDim, kStateDim>::Identity();
    A_.template block<kVelDim, kVelDim>(0, 2) = Eigen::Matrix<double, kVelDim, kVelDim>::Identity() * dt;

    // B_d = B * dt
    B_ = Eigen::Matrix<double, kStateDim, kInputDim>::Zero();
    B_.template block<kVelDim, kInputDim>(2, 0) = Eigen::Matrix<double, kVelDim, kInputDim>::Identity() * dt;
}

// ============================================================
// 初始化 OSQP
//
// 决策变量：
//   z = [x_0, x_1, ..., x_N, u_0, u_1, ..., u_{N-1}]
//
// 约束：
//   1. 初始状态等式：4
//   2. 动力学等式：N * 4
//   3. 速度不等式：N * 2       只约束 vx, vy，且 k = 1...N
//   4. 控制不等式：N * 2
// ============================================================
void Mpc::init_solver() {
    const int N = param_.N;
    constexpr int kStateDim = K_STATE_DIM; // 4
    constexpr int kInputDim = K_INPUT_DIM; // 2

    const int nx = kStateDim;
    const int nu = kInputDim;

    n_vars_ = nx * (N + 1) + nu * N;

    // 只对 vx,vy 做状态硬约束，k=1..N
    const int n_eq = nx + N * nx;
    const int n_ineq_state = N * 2;
    const int n_ineq_u = N * nu;
    n_constraints_ = n_eq + n_ineq_state + n_ineq_u;

    H_.resize(n_vars_, n_vars_);
    g_.resize(n_vars_);
    A_lin_.resize(n_constraints_, n_vars_);
    lb_.resize(n_constraints_);
    ub_.resize(n_constraints_);

    g_.setZero();
    lb_.setZero();
    ub_.setZero();
    // ============================================================
    // 构建 Hessian H_（常数）
    // ============================================================
    H_.setZero();
    std::vector<Eigen::Triplet<double>> h_triplets;

    // 状态代价
    for (int k = 0; k <= N; ++k) {
        const auto& Qk = (k == N) ? param_.QN : param_.Q;
        const int offset = k * nx;

        for (int i = 0; i < nx; ++i) {
            h_triplets.emplace_back(offset + i, offset + i, 2.0 * Qk(i, i));
        }
    }

    // 控制代价 + 控制增量代价
    for (int k = 0; k < N; ++k) {
        const int offset = nx * (N + 1) + k * nu;

        for (int i = 0; i < nu; ++i) {
            h_triplets.emplace_back(offset + i, offset + i, 2.0 * param_.R(i, i));
        }

        // Rd：惩罚 u_k - u_{k-1}
        if (k > 0) {
            const int prev_offset = nx * (N + 1) + (k - 1) * nu;

            for (int i = 0; i < nu; ++i) {
                h_triplets.emplace_back(offset + i, offset + i, 2.0 * param_.Rd(i, i));
                h_triplets.emplace_back(prev_offset + i, prev_offset + i, 2.0 * param_.Rd(i, i));
                h_triplets.emplace_back(offset + i, prev_offset + i, -2.0 * param_.Rd(i, i));
                h_triplets.emplace_back(prev_offset + i, offset + i, -2.0 * param_.Rd(i, i));
            }
        }
    }

    H_.setFromTriplets(h_triplets.begin(), h_triplets.end());

    // ============================================================
    // 构建线性约束矩阵 A_lin_（常数）
    // ============================================================
    A_lin_.setZero();
    std::vector<Eigen::Triplet<double>> a_triplets;
    int row = 0;

    // 1. 初始状态等式约束
    for (int i = 0; i < nx; ++i) {
        a_triplets.emplace_back(row, i, 1.0);
        ++row;
    }

    // 2. 动力学等式约束
    for (int k = 0; k < N; ++k) {
        const int xk_offset = k * nx;
        const int xk1_offset = (k + 1) * nx;
        const int uk_offset = nx * (N + 1) + k * nu;

        for (int i = 0; i < nx; ++i) {
            a_triplets.emplace_back(row, xk1_offset + i, 1.0);

            for (int j = 0; j < nx; ++j) {
                if (std::abs(A_(i, j)) > 1e-12) {
                    a_triplets.emplace_back(row, xk_offset + j, -A_(i, j));
                }
            }

            for (int j = 0; j < nu; ++j) {
                if (std::abs(B_(i, j)) > 1e-12) {
                    a_triplets.emplace_back(row, uk_offset + j, -B_(i, j));
                }
            }

            ++row;
        }
    }

    // 3. 状态不等式：只约束 vx,vy，且 k = 1...N
    //    状态布局: [x, y, vx, vy]
    for (int k = 1; k <= N; ++k) {
        const int offset = k * nx;

        // vx
        a_triplets.emplace_back(row, offset + 2, 1.0);
        ++row;

        // vy
        a_triplets.emplace_back(row, offset + 3, 1.0);
        ++row;
    }

    // 4. 控制不等式
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
// ============================================================
bool Mpc::build_problem(const StateVector& x0, double t_now) {
    if (!traj_ || !traj_->valid()) {
        return false;
    }

    const int N = param_.N;
    constexpr int kStateDim = K_STATE_DIM;
    constexpr int kInputDim = K_INPUT_DIM;

    const int nx = kStateDim;
    const int nu = kInputDim;

    // 1. 采样参考轨迹
    std::vector<TarjectoryInterfaces::ReferencePoint> refs;
    traj_->sample_sequence(t_now, param_.dt, N, refs);

    if (refs.size() < static_cast<size_t>(N + 1)) {
        return false;
    }

    // ============================================================
    // 2. 更新梯度 g_
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
        for (int i = 0; i < nx; ++i) {
            lb_(row) = 0.0;
            ub_(row) = 0.0;
            ++row;
        }
    }

    // 3.3 状态不等式：只约束 vx,vy，k = 1...N
    for (int k = 1; k <= N; ++k) {
        // vx
        lb_(row) = param_.x_min(2);
        ub_(row) = param_.x_max(2);
        ++row;

        // vy
        lb_(row) = param_.x_min(3);
        ub_(row) = param_.x_max(3);
        ++row;
    }

    // 3.4 控制不等式
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
bool Mpc::solve(const StateVector& x0, double t_now, InputVector& u_cmd) {
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
    std::vector<InputVector>& predicted_inputs
) {
    // 求解前检查
    if (!x0.allFinite() || !std::isfinite(t_now)) {
        logger::error(logger::controller, "[MPC] invalid input: x0 finite={}, t_now={}", x0.allFinite(), t_now);
        return false;
    }

    // 1. 构建 QP 数据
    if (!build_problem(x0, t_now)) {
        logger::error(logger::controller, "[MPC] build_problem failed");
        return false;
    }

    // 2. 数据检查
    if (!g_.allFinite() || !lb_.allFinite() || !ub_.allFinite()) {
        logger::error(
            logger::controller,
            "[MPC] non-finite QP data: g={}, lb={}, ub={}",
            g_.allFinite(),
            lb_.allFinite(),
            ub_.allFinite()
        );
        return false;
    }

    if (g_.size() != n_vars_ || lb_.size() != n_constraints_ || ub_.size() != n_constraints_) {
        logger::error(
            logger::controller,
            "[MPC] QP dimension mismatch: g={}, lb={}, ub={}, expected vars={}, constraints={}",
            g_.size(),
            lb_.size(),
            ub_.size(),
            n_vars_,
            n_constraints_
        );
        return false;
    }

    if ((lb_.array() > ub_.array()).any()) {
        logger::error(logger::controller, "[MPC] invalid bounds: some lower bounds exceed upper bounds");
        return false;
    }

    // 3. 更新 OSQP
    if (!solver_.updateGradient(g_) || !solver_.updateBounds(lb_, ub_)) {
        logger::error(logger::controller, "[MPC] OSQP data update failed");
        return false;
    }

    // 4. 求解
    const auto solve_status = solver_.solveProblem();
    if (solve_status != OsqpEigen::ErrorExitFlag::NoError) {
        logger::error(logger::controller, "[MPC] OSQP solve failed, status={}", static_cast<int>(solve_status));
        return false;
    }

    // 重要：必须检查 OSQP 是否真的求解成功
    const auto solver_status = solver_.getStatus();
    if (solver_status != OsqpEigen::Status::Solved && solver_status != OsqpEigen::Status::SolvedInaccurate) {
        logger::error(logger::controller, "[MPC] OSQP not solved, status={}", static_cast<int>(solver_status));
        return false;
    }

    // 5. 提取解
    Eigen::VectorXd solution = solver_.getSolution();

    if (solution.size() != n_vars_) {
        logger::error(logger::controller, "[MPC] invalid solution size: got={}, expected={}", solution.size(), n_vars_);
        return false;
    }

    if (!solution.allFinite()) {
        logger::error(logger::controller, "[MPC] solution contains NaN/Inf");
        return false;
    }

    const int N = param_.N;
    constexpr int kStateDim = K_STATE_DIM;
    constexpr int kInputDim = K_INPUT_DIM;

    const int nx = kStateDim;
    const int nu = kInputDim;

    // 5.1 当前控制量
    u_cmd = solution.segment<kInputDim>(nx * (N + 1));

    // 5.2 预测状态和控制序列
    predicted_states.clear();
    predicted_inputs.clear();

    for (int k = 0; k <= N; ++k) {
        predicted_states.push_back(solution.segment<kStateDim>(k * nx));
    }

    for (int k = 0; k < N; ++k) {
        predicted_inputs.push_back(solution.segment<kInputDim>(nx * (N + 1) + k * nu));
    }

    // 5.3 再次检查
    if (!u_cmd.allFinite()) {
        logger::error(logger::controller, "[MPC] current control contains NaN/Inf: {}", u_cmd.transpose());
        return false;
    }

    for (int k = 0; k <= N; ++k) {
        if (!predicted_states[k].allFinite()) {
            logger::error(logger::controller, "[MPC] predicted state {} contains NaN/Inf", k);
            return false;
        }
    }

    for (int k = 0; k < N; ++k) {
        if (!predicted_inputs[k].allFinite()) {
            logger::error(logger::controller, "[MPC] predicted input {} contains NaN/Inf", k);
            return false;
        }
    }

    // 6. 保存上一次控制量
    param_.u_prev = u_cmd;

    return true;
}

} // namespace control