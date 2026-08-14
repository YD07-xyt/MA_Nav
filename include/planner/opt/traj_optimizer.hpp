/*
    ESDF + 5 阶(quintic)轨迹优化器,基于新框架 SplineOptimizer。

    与旧 `include/traj_optimize/traj_opt.hpp` 的区别:
      - 旧:自写代价 + 自写梯度反传 + utils/SplineTrajectory.hpp 的 cubic 样条;
      - 新:只写 TimeCost / IntegralCost,样条求解、能量、drift 梯度反传、
            时间映射(tau <-> T)全部由 SplineOptimizer 负责。
    默认 SplineType = QuinticSplineND<2> -> 位置/速度/加速度 C2 连续(5 阶)。
    平滑 = rho_energy(jerk^2 能量)+ 边界加速度归零;
    动力学可行 = max_v / max_a (可选 max_jerk) Smoothed L1 软约束;
    避障 = ESDF Smoothed L1 惩罚 + 正交梯度投影 + 谷底增强,
           设计参考 include/traj_optimize/minco_optimizer.hpp 的 computeObstacleCost。

    参数来源:Opt::OptimizerParams 结构体默认值,可通过 yaml 键 opt.* 覆盖
    (见 include/config.hpp 与 config/global_planning.yaml)。
*/

#ifndef OPT_TRAJ_OPTIMIZER_HPP
#define OPT_TRAJ_OPTIMIZER_HPP

#include "SplineTrajectory/SplineOptimizer.hpp"
#include "map/grid_map.hpp"
#include "utils/lbfgs.hpp"

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <limits>
#include <utility>

namespace Opt
{

    /// 优化器参数。默认值已适配仓库里 2D ESDF + quintic 场景。
    struct OptimizerParams
    {
        double total_time = 10.0;   // 轨迹总时间初值(会被 rho_T 继续压缩)
        int piece_num = 0;          // 0 = 自动按 piece_len 弧长重采样;>0 则固定段数重采样
        double piece_len = 1.0;     // 自动重采样时每段目标弧长(m)。A* optimizePath 只剪枝不限制点数,
                                    // 直接当段点会导致决策变量几百维、L-BFGS 不收敛
        int max_pieces = 40;        // 自动重采样时段数上限(限制决策变量维度)
        double rho_energy = 5.0;    // jerk^2 能量权重 -> 平滑位置/速度/加速度
        double rho_T = 100.0;       // 总时间惩罚权重
        double rho_obs = 1000.0;    // ESDF 碰撞惩罚权重(Smoothed L1 恒定梯度;旧配置 rho_collision=1000)
        double safe_threshold = 0.45;// 安全距离阈值(m)
        double max_v = 3.0;         // 最大线速度(m/s),动力学可行
        double rho_v = 100.0;       // 速度超限惩罚权重
        double max_a = 3.0;         // 最大加速度(m/s^2),动力学可行
        double rho_a = 50.0;        // 加速度超限惩罚权重
        double max_j = 0.0;         // 最大 jerk,<=0 禁用
        double rho_j = 0.0;         // jerk 超限惩罚权重
        int int_K = 32;             // 每段梯形积分采样点数(密集障碍调大)
        bool check_gradient = false;// 优化前做一次解析/数值梯度校验

        // L-BFGS 参数
        int mem_size = 256;
        int max_iter = 10000;
        int past = 3;
        double g_epsilon = 1e-6;
        double delta = 1e-5;
        double min_step = 1e-32;
    };

    /// 逐积分点代价:ESDF 避障 + 速度/加速度/jerk 限幅。
    /// 只需写 gp/gv/ga/gj,框架自动完成 drift 反传到时间变量。
    ///
    /// 避障部分参考 minco_optimizer.hpp::computeObstacleCost:
    ///   - Smoothed L1 惩罚:违反量超过 1cm 后恒定梯度(模拟硬约束),
    ///     避免二次惩罚在贴墙时梯度衰减导致的贴墙/穿障局部最优;
    ///   - 正交梯度投影:剔除切线分量,只留横向推力;
    ///   - 谷底增强:投影后梯度过小(垂直穿墙/势能谷底)先 sqrt 增强模长,
    ///     仍退化则用法线方向兜底(不用原始 ESDF 梯度——它沿轨迹纵向,
    ///     经 drift 项会拉爆时间变量,见 docs/CHANGELOG.md 的 -1009 教训)。
    struct EsdfIntegralCost
    {
        std::shared_ptr<grid_map::GridMap> map;
        OptimizerParams params;

        /**
         * @brief 正部 Smoothed L1 惩罚函数(与 minco_optimizer.hpp::positiveSmoothedL1 一致)。
         *
         * 对违反量 x(>0 表示违反约束)施加惩罚,数学形式:
         *   x >= pe:      f = x - 0.5*pe,  df = 1.0    -> 线性惩罚,恒定梯度(=1),
         *                                                 违反越深推力越强,模拟硬约束;
         *   0 < x < pe:   f = (f4c*x + f3c)*x^3,       -> 四次多项式,保证 f(0)=f'(0)=0,
         *                 df = (d3c*x + d2c)*x^2          f(pe)=pe-0.5*pe, f'(pe)=1,即 C2 连续;
         *   x <= 0:       f = 0, df = 0                 -> 无违反不惩罚。
         *
         * 相比二次惩罚 cv^2:二次惩罚在贴墙时梯度随违反量线性衰减,优化器会停在
         * “梯度消失”的贴墙/穿障局部最优;Smoothed L1 在违反超过 pe(1cm)后恒定
         * 梯度,彻底消除该问题(仓库 CHANGELOG 实测)。
         *
         * @param x  违反量(正值表示违反约束)
         * @param f  输出:惩罚值
         * @param df 输出:惩罚对 x 的导数 d f / d x
         */
        static inline void positiveSmoothedL1(double x, double &f, double &df)
        {
            const double pe = 1.0e-2;
            const double f3c = 1.0 / (pe * pe);
            const double f4c = -0.5 * f3c / pe;
            const double d2c = 3.0 * f3c;
            const double d3c = 4.0 * f4c;

            if (x < pe)
            {
                if (x > 0.0)
                {
                    f = (f4c * x + f3c) * x * x * x;
                    df = (d3c * x + d2c) * x * x;
                }
                else
                {
                    f = 0.0;
                    df = 0.0;
                }
            }
            else
            {
                f = x - 0.5 * pe;
                df = 1.0;
            }
        }

        /**
         * @brief 逐积分点代价回调(由 SplineOptimizer 在每段每个梯形积分采样点调用)。
         *
         * 框架在每个采样点提供位置 p / 速度 v / 加速度 a / jerk j / snap s,
         * 本函数返回该点瞬时代价,并把代价对各状态的偏导写入 gp/gv/ga/gj/gs
         * (gt 为对时间的显式偏导,通常不需要)。框架自动完成:
         *   1) 梯形积分加权(端点 0.5,内部 1.0);
         *   2) 通过样条基函数把 gp/gv/ga/gj 反传到多项式系数;
         *   3) drift 项(gp·v + gv·a + ga·j + gj·s)自动成为对段时间的偏导,
         *      无需手写 grad_time。
         *
         * 代价组成(全部为 Smoothed L1 软约束,连续可微):
         *   1. ESDF 避障:viola = safe_threshold - sdf,惩罚 = rho_obs * SL1(viola);
         *      位置梯度 = -rho_obs * dSL1 * grad_sdf,并做:
         *        ① 正交投影:剔除沿速度(轨迹切线)方向的分量,只留横向推力,
         *           防止纵向力经 drift 项拉爆时间变量(L-BFGS -1009 教训);
         *        ② 谷底增强:投影后梯度过小(垂直穿墙/两侧障碍抵消)时,
         *           先 sqrt 增强模长,仍退化则用法线方向兜底。
         *   2. 速度限幅:viola = |v|^2 - max_v^2,惩罚 = rho_v * SL1(viola),
         *      gv = rho_v * dSL1 * 2v(只罚超速,不约束最低速);
         *   3. 加速度限幅:同上,基于 |a|^2 - max_a^2(5 阶才有连续可罚的 a);
         *   4. jerk 限幅(可选):同上,基于 |j|^2 - max_j^2。
         *
         * @param point 积分点元信息(段索引/局部时间/梯形权重等,一般用不到)
         * @param p/v/a/j/s 该采样点的位置/速度/加速度/jerk/snap
         * @param gp/gv/ga/gj/gs 输出:代价对各状态的偏导(初始为 0,累加即可)
         * @param gt 输出:代价对时间的显式偏导(通常保持 0)
         * @return 该采样点的瞬时代价(框架负责积分加权)
         */
        double operator()(const SplineTrajectory::IntegralPointInfo & /*info*/,
                          const Eigen::Vector2d &p, const Eigen::Vector2d &v, const Eigen::Vector2d &a,
                          const Eigen::Vector2d &j, const Eigen::Vector2d & /*s*/,
                          Eigen::Vector2d &gp, Eigen::Vector2d &gv, Eigen::Vector2d &ga,
                          Eigen::Vector2d &gj, Eigen::Vector2d & /*gs*/, double & /*gt*/) const
        {
            double cost = 0.0;
            const double vn = v.norm();
            constexpr double kGradNormThreshold = 0.5; // 梯度增强阈值(minco 同款)
            constexpr double kMinGradNorm = 1e-6;      // 最小梯度模长

            // ---- 1. ESDF 避障(Smoothed L1 + 正交投影 + 谷底增强)----
            double sdf = 0.0;
            Eigen::Vector2d grad_sdf = Eigen::Vector2d::Zero();
            if (!map->getDistanceAndGradient(p, sdf, grad_sdf))
            {
                // 出图:按最深违反处理(堵死"地图外免费空间"漏洞,比 minco 的
                // continue 更严格);推力方向改为朝地图中心回推,解决原始实现
                // "梯度为 0 无推力"的缺陷。
                sdf = -params.safe_threshold;
                const Eigen::Vector2d center = map->getOrigin() + 0.5 * map->getMapSize();
                grad_sdf = center - p;
                if (grad_sdf.norm() > 1e-9)
                    grad_sdf.normalize();
            }

            const double viola = params.safe_threshold - sdf;
            double penalty = 0.0, dpen = 0.0;
            positiveSmoothedL1(viola, penalty, dpen);
            if (penalty > 0.0)
            {
                cost += params.rho_obs * penalty;

                Eigen::Vector2d grad_obs = grad_sdf; // 指向远离障碍

                // ① 正交投影:剔除沿速度(切线)方向的分量
                if (vn > kMinGradNorm)
                {
                    const Eigen::Vector2d tangent = v / vn;
                    grad_obs -= grad_obs.dot(tangent) * tangent;
                }

                // ② 谷底增强
                const double gn = grad_obs.norm();
                if (gn < kGradNormThreshold && gn > kMinGradNorm)
                {
                    grad_obs = grad_obs / gn * std::sqrt(gn); // sqrt 增强,保留方向
                }
                else if (gn <= kMinGradNorm)
                {
                    // 投影后梯度几乎为零(垂直穿墙):用法线方向兜底
                    if (vn > kMinGradNorm)
                    {
                        const Eigen::Vector2d tangent = v / vn;
                        Eigen::Vector2d lateral(-tangent.y(), tangent.x());
                        if (grad_sdf.dot(lateral) < 0.0)
                            lateral = -lateral;
                        grad_obs = lateral * kGradNormThreshold;
                    }
                }

                // 链式法则:∂cost/∂p = rho * ∂penalty/∂viola * (-grad_sdf)
                gp = params.rho_obs * (-dpen) * grad_obs;
            }

            // ---- 2~4. 速度/加速度/jerk 限幅(Smoothed L1,minco 同款) ----
            double pv = 0.0, dpv = 0.0;
            positiveSmoothedL1(v.squaredNorm() - params.max_v * params.max_v, pv, dpv);
            if (pv > 0.0)
            {
                cost += params.rho_v * pv;
                gv = params.rho_v * dpv * 2.0 * v;
            }

            if (params.rho_a > 0.0)
            {
                double pa = 0.0, dpa = 0.0;
                positiveSmoothedL1(a.squaredNorm() - params.max_a * params.max_a, pa, dpa);
                if (pa > 0.0)
                {
                    cost += params.rho_a * pa;
                    ga = params.rho_a * dpa * 2.0 * a;
                }
            }

            if (params.rho_j > 0.0 && params.max_j > 0.0)
            {
                double pj = 0.0, dpj = 0.0;
                positiveSmoothedL1(j.squaredNorm() - params.max_j * params.max_j, pj, dpj);
                if (pj > 0.0)
                {
                    cost += params.rho_j * pj;
                    gj = params.rho_j * dpj * 2.0 * j;
                }
            }

            return cost;
        }
    };

    /**
     * @brief 时间代价:最小化轨迹总时长。
     *
     * 对每段时间 T_i 施加线性惩罚 rho_T * T_i,梯度恒为 rho_T。
     * 框架通过默认 QuadInvTimeMap 把时间决策变量映射为无约束 tau(tau -> T
     * 保证 T>0),并把本梯度经 dT/dtau 链式法则自动转到决策变量。
     * 作用是压住优化器把轨迹时长拉长的倾向,与 rho_energy 形成
     * “快 vs 平滑”的平衡。
     */
    struct TimeCost
    {
        double rho_T = 100.0; ///< 总时长惩罚权重(yaml: opt.rho_T)

        /**
         * @brief 时间代价回调。
         * @param Ts   当前各段时间(秒),size = 段数 N
         * @param grad 输出:对每段时间的偏导(缓冲区已 resize/置零,直接写值)
         * @return 代价 = rho_T * sum(Ts)
         */
        double operator()(const std::vector<double> &Ts, Eigen::VectorXd &grad) const
        {
            const double total = std::accumulate(Ts.begin(), Ts.end(), 0.0);
            for (Eigen::Index i = 0; i < grad.size(); ++i)
                grad(i) = rho_T;
            return rho_T * total;
        }
    };

    /**
     * @brief ESDF + quintic 轨迹优化器(2D)。
     *
     * 完整流程:输入 A* 路径点 -> 弧长重采样定段 -> 构造 SplineOptimizer
     * (默认 QuinticSplineND<2>,5 阶样条,C2 连续)-> L-BFGS 最小化
     * (ESDF 避障 + 速度/加速度限幅 + jerk^2 能量 + 总时长)-> 输出样条轨迹。
     *
     * 与旧 TrajOpt::TrajectoryOptimizer 的接口对应:plan/getTrajectory/
     * sampleTrajectory/evaluate/setStartVel/setEndVel,便于直接替换。
     */
    class EsdfTrajectoryOptimizer
    {
    public:
        using Vector2d = Eigen::Vector2d;
        // quintic:每段 6 个系数 -> 位置/速度/加速度 C2 连续
        using PPoly2D = SplineTrajectory::PPolyND<2, 6>;

        EsdfTrajectoryOptimizer() = default;

        /**
         * @brief 构造优化器。
         * @param map    2D ESDF 地图(提供 getDistanceAndGradient 查询)
         * @param path   初始路径(A* 剪枝后的路径点,plan 内部会再按 piece_len 重采样)
         * @param params 优化参数(结构体默认值 + yaml opt.* 覆盖)
         */
        EsdfTrajectoryOptimizer(std::shared_ptr<grid_map::GridMap> map,
                                std::vector<Vector2d> path,
                                const OptimizerParams &params = OptimizerParams())
            : map_(std::move(map)), path_(std::move(path)), params_(params) {}

        void setMap(std::shared_ptr<grid_map::GridMap> map) { map_ = std::move(map); }
        void setPath(const std::vector<Vector2d> &path) { path_ = path; }
        void setParams(const OptimizerParams &params) { params_ = params; }
        void setStartVel(const Vector2d &v) { start_vel_ = v; } ///< 起始速度(会投影到路径首段方向)
        void setEndVel(const Vector2d &v) { end_vel_ = v; }     ///< 末端速度(默认沿末段方向小速度)

        const std::string &lastError() const { return err_; } ///< 最近一次失败的诊断信息

        const PPoly2D &getTrajectory() const { return trajectory_; } ///< 优化结果样条(quintic PPoly)

        /**
         * @brief 执行轨迹优化。
         *
         * 步骤:
         *   1. 校验地图/路径非空;
         *   2. 按 piece_len 弧长把路径重采样到有限段数(避免 A* 密点导致决策变量爆炸);
         *   3. 组装 ProblemDefinition(段时长初值 = total_time/N、路径点、边界条件:
         *      速度取 resolveStartVel/resolveEndVel,加速度归零,jerk 归零钉死欠定自由度);
         *   4. prepareContext 校验并布局决策变量[每段 tau, 内部点(端点固定)];
         *   5. 组装 EvaluateSpec = TimeCost + EsdfIntegralCost(均为 lvalue,常驻);
         *   6. 生成初始猜测并(可选)跑 checkGradients 验证解析梯度;
         *   7. L-BFGS 热路径循环(evaluatePrepared 免重复解码);
         *      达到最大迭代时警告但接受当前解(由外部硬净空复核兜底);
         *   8. synchronizeWorkingState 重建最终样条并拷贝到 trajectory_。
         *
         * @return 是否成功(失败原因见 lastError())
         */
        bool plan()
        {
            err_.clear();
            if (!map_ || path_.size() < 2)
            {
                err_ = "map is null or path has fewer than 2 points";
                return false;
            }

            // 段数:显式 piece_num > 0 用固定段数;否则按 piece_len 弧长自动定段(上限 max_pieces)。
            // A* 的 optimizePath 只做折线剪枝、不限制点数(实测 21m 路径仍 254 点),
            // 直接当 quintic 段点 -> 决策变量 ~760 维 + 每段 0.03s 的五阶系数病态,
            // L-BFGS 打满 max_iter 不收敛(见运行日志 opt-fail)。
            int pieces = params_.piece_num;
            if (pieces <= 0)
            {
                const double len = pathLength(path_);
                const double target = std::max(params_.piece_len, 1e-3);
                pieces = std::max(2, std::min(params_.max_pieces,
                                              static_cast<int>(std::ceil(len / target))));
            }
            const std::vector<Vector2d> pts = resamplePath(path_, pieces);
            const int N = static_cast<int>(pts.size()) - 1;
            if (N < 1)
            {
                err_ = "empty trajectory";
                return false;
            }

            using Optimizer = SplineTrajectory::SplineOptimizer<2>; // 默认 QuinticSplineND<2>

            Optimizer optimizer;
            Optimizer::OptimizerConfig config;
            config.rho_energy = params_.rho_energy;       // jerk^2 平滑
            config.integral_num_steps = params_.int_K;    // ESDF 采样密度
            optimizer.setConfig(config);

            Optimizer::ProblemDefinition problem;
            problem.time_segments.assign(N, params_.total_time / N);
            problem.waypoints.resize(N + 1, 2);
            for (int i = 0; i <= N; ++i)
                problem.waypoints.row(i) = pts[i].transpose();
            problem.start_time = 0.0;

            SplineTrajectory::BoundaryConditions<2> bc;
            bc.start_velocity = resolveStartVel(pts);
            bc.end_velocity = resolveEndVel(pts);
            bc.start_acceleration = Vector2d::Zero();     // 平滑起步
            bc.end_acceleration = Vector2d::Zero();       // 平滑停车
            // start_jerk / end_jerk 保持默认 0(钉死 quintic 欠定的 2 个自由度)
            problem.bc = bc;

            Optimizer::OptimizationContext ctx;
            auto status = optimizer.prepareContext(problem, ctx);
            if (!status)
            {
                err_ = status.message;
                return false;
            }

            EsdfIntegralCost integral_cost{map_, params_};
            TimeCost time_cost{params_.rho_T};
            auto spec = Optimizer::makeEvaluateSpec(time_cost, integral_cost); // 必须 lvalue

            Eigen::VectorXd x = optimizer.generateInitialGuess(ctx);
            Eigen::VectorXd grad(x.size());

            if (params_.check_gradient)
            {
                auto gc = optimizer.checkGradients(ctx, x, spec);
                if (!gc.valid)
                {
                    err_ = gc.makeReport();
                    return false;
                }
            }

            lbfgs::lbfgs_parameter_t lp;
            lp.mem_size = params_.mem_size;
            lp.past = params_.past;
            lp.g_epsilon = params_.g_epsilon;
            lp.min_step = params_.min_step;
            lp.delta = params_.delta;
            lp.max_iterations = params_.max_iter;

            using SpecType = decltype(spec);
            struct Callback
            {
                Optimizer *opt;
                Optimizer::OptimizationContext *ctx;
                SpecType *spec_ptr;
            };
            Callback cb{&optimizer, &ctx, &spec};
            auto eval_fn = [](void *instance, const Eigen::VectorXd &xx, Eigen::VectorXd &g) -> double {
                auto *c = static_cast<Callback *>(instance);
                return c->opt->evaluatePrepared(*c->ctx, xx, g, *c->spec_ptr);
            };

            double final_cost = 0.0;
            const int ret = lbfgs::lbfgs_optimize(x, final_cost, eval_fn,
                                                  nullptr, nullptr, &cb, lp);
            if (ret == lbfgs::LBFGSERR_MAXIMUMITERATION)
            {
                // 达到最大迭代:警告但不直接放弃,用当前解 + 硬净空复核兜底
                err_ = "L-BFGS reached max iterations, accepting current solution";
            }
            else if (ret < 0)
            {
                err_ = lbfgs::lbfgs_strerror(ret);
                return false;
            }

            auto sync = optimizer.synchronizeWorkingState(ctx, x);
            if (!sync)
            {
                err_ = sync.message;
                return false;
            }

            trajectory_ = optimizer.getWorkingSpline(ctx).getTrajectory();
            return true;
        }

        /**
         * @brief 按固定时间步长采样优化轨迹的位置点(供可视化/碰撞复核/MPC 参考)。
         * @param dt 采样时间步长(s)。常用:0.1(可视化)、0.02(硬净空复核)
         * @return 位置点序列(含终点,保证末点与轨迹末端一致)
         */
        std::vector<Vector2d> sampleTrajectory(double dt = 0.1) const
        {
            std::vector<Vector2d> out;
            if (!trajectory_.isInitialized())
                return out;

            const double start = trajectory_.getStartTime();
            const double end = trajectory_.getEndTime();
            for (double t = start; t <= end; t += dt)
                out.push_back(trajectory_.evaluate(t, SplineTrajectory::Deriv::Pos));

            const Vector2d last = trajectory_.evaluate(end, SplineTrajectory::Deriv::Pos);
            if (out.empty() || (out.back() - last).norm() > 1e-9)
                out.push_back(last);
            return out;
        }

        /**
         * @brief 轨迹指标(诊断用):
         *  max_velocity / min_velocity / avg_velocity  峰值/最低/平均速度(动力学可行验证)
         *  max_acceleration                          峰值加速度
         *  min_clearance                            到最近障碍的最小净空(安全验证,应 > hard_clearance)
         *  duration                                 总时长
         *  trajectory_energy                        acc^2 积分(控制能量,与旧接口字段对齐)
         *  rms_jerk                                 jerk^2 RMS(5 阶平滑度)
         *  path_deviation                           相对输入路径的平均偏差
         */
        struct Metrics
        {
            double max_velocity = 0.0;
            double min_velocity = std::numeric_limits<double>::max();
            double avg_velocity = 0.0;
            double max_acceleration = 0.0;
            double min_clearance = std::numeric_limits<double>::max();
            double duration = 0.0;
            double trajectory_energy = 0.0; // acc^2 积分(与旧接口对齐)
            double rms_jerk = 0.0;          // jerk^2 RMS(quintic 平滑度)
            double path_deviation = 0.0;
        };

        /**
         * @brief 密集采样评估轨迹指标(0.01s 步长,见 Metrics 字段说明)。
         * @param dt 采样步长,默认 0.01s
         * @return 指标结构体(轨迹未初始化时全零/极大值)
         */
        Metrics evaluate(double dt = 0.01) const
        {
            Metrics m;
            if (!trajectory_.isInitialized())
                return m;

            m.duration = trajectory_.getDuration();
            double jerk_acc = 0.0;
            double vel_sum = 0.0;
            int count = 0;
            const double start = trajectory_.getStartTime();
            const double end = trajectory_.getEndTime();

            for (double t = start; t <= end; t += dt)
            {
                const Vector2d pos = trajectory_.evaluate(t, SplineTrajectory::Deriv::Pos);
                const Vector2d vel = trajectory_.evaluate(t, SplineTrajectory::Deriv::Vel);
                const Vector2d acc = trajectory_.evaluate(t, SplineTrajectory::Deriv::Acc);
                const Vector2d jrk = trajectory_.evaluate(t, SplineTrajectory::Deriv::Jerk);

                const double vn = vel.norm();
                m.max_velocity = std::max(m.max_velocity, vn);
                m.min_velocity = std::min(m.min_velocity, vn);
                vel_sum += vn;
                m.max_acceleration = std::max(m.max_acceleration, acc.norm());
                m.trajectory_energy += acc.squaredNorm() * dt;
                jerk_acc += jrk.squaredNorm();
                ++count;

                if (map_)
                    m.min_clearance = std::min(m.min_clearance, map_->getDistance(pos));

                double min_d = std::numeric_limits<double>::max();
                for (const auto &wp : path_)
                    min_d = std::min(min_d, (pos - wp).norm());
                m.path_deviation += min_d;
            }

            if (count > 0)
            {
                m.rms_jerk = std::sqrt(jerk_acc / count);
                m.avg_velocity = vel_sum / count;
                m.path_deviation /= count;
            }
            return m;
        }

    private:
        /** @brief 折线路径总弧长(用于按 piece_len 自动定段数)。 */
        static double pathLength(const std::vector<Vector2d> &path)
        {
            double len = 0.0;
            for (size_t i = 1; i < path.size(); ++i)
                len += (path[i] - path[i - 1]).norm();
            return len;
        }

        /**
         * @brief 按弧长均匀重采样路径到 piece_num 段(piece_num+1 个点,含首尾)。
         *
         * A* optimizePath 只做贪心剪枝不限制点数,直接作为 quintic 段点会导致
         * 决策变量维度爆炸 + 每段时间过短的数值病态;这里把路径压到几十段,
         * 每段弧长 ≈ 总长/piece_num,端点保持不变。
         * 若 piece_num <= 0 或 >= 原段数(路径本来就稀疏),原样返回。
         *
         * @param path      输入折线(弧长单调递增的路径点序列)
         * @param piece_num 目标段数
         * @return 重采样后的段点序列(首尾 = 输入首尾)
         */
        static std::vector<Vector2d> resamplePath(const std::vector<Vector2d> &path, int piece_num)
        {
            const int n = static_cast<int>(path.size());
            if (piece_num <= 0 || piece_num >= n - 1)
                return path;

            std::vector<double> cum(n, 0.0);
            for (int i = 1; i < n; ++i)
                cum[i] = cum[i - 1] + (path[i] - path[i - 1]).norm();
            const double total = cum.back();
            if (total < 1e-6)
                return path;

            std::vector<Vector2d> out(static_cast<size_t>(piece_num) + 1);
            out.front() = path.front();
            out.back() = path.back();
            int seg = 0;
            for (int k = 1; k < piece_num; ++k)
            {
                const double target = total * k / piece_num;
                while (seg < n - 2 && cum[seg + 1] < target)
                    ++seg;
                const double seg_len = cum[seg + 1] - cum[seg];
                const double ratio = seg_len > 1e-9 ? (target - cum[seg]) / seg_len : 0.0;
                out[k] = path[seg] + ratio * (path[seg + 1] - path[seg]);
            }
            return out;
        }

        /**
         * @brief 解析起始速度边界。
         * 把外部给定的 start_vel 投影到路径首段方向:方向一致则保留沿路径分量
         * (丢弃横向分量,避免样条侧向扭曲);反向(换目标/掉头)退化为沿路径
         * 小速度起步,避免样条被迫高速掉头导致能量爆炸/线搜索失败。
         * 未设置时默认沿首段方向 0.5 m/s 起步。
         */
        Vector2d resolveStartVel(const std::vector<Vector2d> &pts) const
        {
            Vector2d dir = pts[1] - pts[0];
            const double len = dir.norm();
            if (len < 1e-6)
                return Vector2d::Zero();
            dir /= len;

            if (start_vel_.norm() > 0.01)
            {
                const double v_along = start_vel_.dot(dir);
                return (v_along > 0.01) ? dir * v_along : dir * 0.5;
            }
            return dir * 0.5;
        }

        /**
         * @brief 解析末端速度边界。未设置时默认沿末段方向 0.1 m/s(接近停车,
         * 末端加速度已归零,平滑收尾)。
         */
        Vector2d resolveEndVel(const std::vector<Vector2d> &pts) const
        {
            if (end_vel_.norm() > 0.01)
                return end_vel_;
            const Vector2d dir = (pts.back() - pts[pts.size() - 2]).normalized();
            return dir * 0.1;
        }

        std::shared_ptr<grid_map::GridMap> map_;
        std::vector<Vector2d> path_;
        OptimizerParams params_;
        Vector2d start_vel_ = Vector2d::Zero();
        Vector2d end_vel_ = Vector2d::Zero();
        PPoly2D trajectory_;
        std::string err_;
    };

} // namespace Opt

#endif // OPT_TRAJ_OPTIMIZER_HPP
