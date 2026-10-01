#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <OsqpEigen/Solver.hpp>

#include <memory>
#include <vector>

namespace control {

// ============================================================
// 轨迹接口
// ============================================================
class TarjectoryInterfaces {
public:
    static constexpr int K_STATE_DIM = 4; // x, y, vx, vy
    static constexpr int K_INPUT_DIM = 2; // ax, ay

    using StateVector = Eigen::Matrix<double, K_STATE_DIM, 1>;
    using InputVector = Eigen::Matrix<double, K_INPUT_DIM, 1>;

    struct ReferencePoint {
        double time = 0.0;
        StateVector state = StateVector::Zero();
        InputVector input = InputVector::Zero();
    };

    virtual ~TarjectoryInterfaces() = default;

    virtual bool valid() const = 0;
    virtual double duration() const = 0;
    virtual bool sample(double t, ReferencePoint& ref) const = 0;
    // 初始化时使用：全局搜索最近点
    virtual double nearest_time(const Eigen::Vector2d& pos) const = 0;

    // 每帧使用：局部搜索，避免全局扫描
    virtual double update_track_time(const Eigen::Vector2d& pos, double hint) const = 0;
    virtual bool sample_sequence(double t_start, double dt, int N, std::vector<ReferencePoint>& refs) const = 0;
};

// ============================================================
// MPC
// ============================================================
class Mpc {
public:
    static constexpr int K_STATE_DIM = 4; // x, y, vx, vy
    static constexpr int K_INPUT_DIM = 2; // ax, ay
    using StateVector = Eigen::Matrix<double, K_STATE_DIM, 1>;
    using InputVector = Eigen::Matrix<double, K_INPUT_DIM, 1>;
    using TrajectoryPtr = std::shared_ptr<TarjectoryInterfaces>;

    struct Param {
        int N = 20;
        double dt = 0.05;

        // 状态权重
        Eigen::Matrix<double, K_STATE_DIM, K_STATE_DIM> Q = Eigen::Matrix<double, K_STATE_DIM, K_STATE_DIM>::Zero();
        Eigen::Matrix<double, K_STATE_DIM, K_STATE_DIM> QN = Eigen::Matrix<double, K_STATE_DIM, K_STATE_DIM>::Zero();

        // 控制权重
        Eigen::Matrix<double, K_INPUT_DIM, K_INPUT_DIM> R = Eigen::Matrix<double, K_INPUT_DIM, K_INPUT_DIM>::Zero();
        Eigen::Matrix<double, K_INPUT_DIM, K_INPUT_DIM> Rd = Eigen::Matrix<double, K_INPUT_DIM, K_INPUT_DIM>::Zero();

        // 状态/控制限幅
        StateVector x_min = StateVector::Constant(-10.0);
        StateVector x_max = StateVector::Constant(10.0);
        InputVector u_min = InputVector::Constant(-3.0);
        InputVector u_max = InputVector::Constant(3.0);

        // 上一个控制量，用于 Rd 项
        InputVector u_prev = InputVector::Zero();
    };

    explicit Mpc(const Param& param);
    ~Mpc() = default;

    void set_trajectory(TrajectoryPtr traj);

    bool solve(const StateVector& x0, double t_now, InputVector& u_cmd);

    bool solve(
        const StateVector& x0,
        double t_now,
        InputVector& u_cmd,
        std::vector<StateVector>& predicted_states,
        std::vector<InputVector>& predicted_inputs
    );

private:
    void discretize();
    void init_solver();
    bool build_problem(const StateVector& x0, double t_now);

    Param param_;
    TrajectoryPtr traj_;

    // 离散模型
    Eigen::Matrix<double, K_STATE_DIM, K_STATE_DIM> A_;
    Eigen::Matrix<double, K_STATE_DIM, K_INPUT_DIM> B_;

    // OSQP
    OsqpEigen::Solver solver_;

    // QP 数据
    Eigen::SparseMatrix<double> H_;
    Eigen::VectorXd g_;
    Eigen::SparseMatrix<double> A_lin_;
    Eigen::VectorXd lb_;
    Eigen::VectorXd ub_;

    int n_vars_ = 0;
    int n_constraints_ = 0;
};

} // namespace control