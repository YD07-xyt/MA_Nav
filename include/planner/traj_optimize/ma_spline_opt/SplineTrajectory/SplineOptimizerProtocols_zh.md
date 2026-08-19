# SplineOptimizer 协议参考

本文档记录 `SplineOptimizer` 使用的可调用对象(callable)与映射器(mapper)协议。

这些只是参考接口:

- 用户自定义的函数对象(functor)和 lambda **不需要**继承它们
- 只需满足 `SplineOptimizer` 的类型特征(type traits)所检查的签名即可

当前优化器 API 还使用几个小的集成类型:

- `OptimizationContext`:调用方持有的"已准备问题 + 可变运行时状态"
- `EvaluateSpec`:绑定借用的代价函数对象与执行器(executor)
- `ErrorCode`:设置与求值阶段的分类化失败码
- `Status`:`setup/validation` 风格 API 的返回值
- `EvaluationResult`:`evaluate(...)` 的返回值
- `GradientCheckResult`:`checkGradients(...)` 的返回值

典型流程:

1. 创建并初始化一个 `SplineOptimizer`
2. 每个求值上下文创建一个 `OptimizationContext`
3. 用 `makeEvaluateSpec(...)` 构建 `EvaluateSpec`
4. 调用 `evaluate(...)` 并检查 `EvaluationResult::ok`

`EvaluateSpec` 通过"类引用"的存储方式借用代价对象,因此传给
`makeEvaluateSpec(...)` 和 `with...Cost(...)` 的代价函数对象必须是左值
(lvalue),且生命周期要长于 spec 本身。

## TimeMap 协议

```cpp
struct TimeMapProtocol
{
    // 无约束变量 'tau' -> 物理时间 'T'
    double toTime(double tau) const;

    // 物理时间 'T' -> 无约束变量 'tau'
    double toTau(double T) const;

    // 链式法则: dCost/dtau = (dCost/dT) * (dT/dtau)
    double backward(double tau, double T, double gradT) const;
};
```

## SpatialMap 协议

全局绝对索引:

- `index = 0`:起点
- `index = 1 ... N - 1`:内部航点
- `index = N`:终点

```cpp
struct SpatialMapProtocol
{
    // 全局索引对应的点,其无约束变量 xi 的维度
    int getUnconstrainedDim(int index) const;

    // 前向: xi (无约束) -> p (物理)
    Eigen::VectorXd toPhysical(const Eigen::VectorXd& xi, int index) const;

    // 反向: p (物理) -> xi (无约束),用于初值猜测
    Eigen::VectorXd toUnconstrained(const Eigen::VectorXd& p, int index) const;

    // 链式法则: dCost/dxi = (dCost/dp) * (dp/dxi)
    Eigen::VectorXd backwardGrad(const Eigen::VectorXd& xi,
                                 const Eigen::VectorXd& grad_p,
                                 int index) const;
};
```

## IntegralCost 协议

积分代价可以选择性地实现 `beginEvaluation() const`。优化器在解码当前决策
向量之后、访问任何积分采样点之前调用它一次。它只用于重置适配器自有的诊断
状态;适配器**不得**借用或保留优化器的工作状态:

```cpp
void beginEvaluation() const;
```

```cpp
struct IntegralCostProtocol
{
    double operator()(const SplineTrajectory::IntegralPointInfo& point,
                      const Eigen::VectorXd& p,
                      const Eigen::VectorXd& v,
                      const Eigen::VectorXd& a,
                      const Eigen::VectorXd& j,
                      const Eigen::VectorXd& s,
                      Eigen::VectorXd& gp,
                      Eigen::VectorXd& gv,
                      Eigen::VectorXd& ga,
                      Eigen::VectorXd& gj,
                      Eigen::VectorXd& gs,
                      double& gt) const;
};
```

## TimeCost 协议

```cpp
struct TimeCostProtocol
{
    double operator()(const std::vector<double>& Ts,
                      Eigen::VectorXd& grad) const;
};
```

## WaypointsCost 协议

```cpp
struct WaypointsCostProtocol
{
    double operator()(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& waypoints,
                      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& grad_q) const;
};
```

## SampleCost 协议

```cpp
struct SampleCostProtocol
{
    template <typename SamplesType>
    double operator()(const SamplesType& samples,
                      Eigen::Matrix<double, 3, Eigen::Dynamic>& grad_p,
                      Eigen::VectorXd& grad_t_global) const;
};
```

`samples` 是记录的积分缓冲区。每个采样点提供:

- `point`:不可变的积分元数据(`segment_index`、`segment_count`、
  `step_index`、`step_count`、`alpha`、`segment_duration`、`step_size`、
  `local_time`、`global_time`,以及精确端点辅助函数)
- `trap_weight`:梯形积分权重因子
- `b_p`:样条系数对应的位置基函数行向量
- `p`、`v`:采样得到的位置与速度

在当前优化器数据模型下,`SampleCost` 是以下量的离散代价:

- 采样位置 `p`
- 采样全局时间 `t_global`

它目前**不**为 `v/a/j/s` 暴露独立的采样反向(backward)通道。

## TrajectoryCost 协议

```cpp
struct TrajectoryCostProtocol
{
    // 具体样条类型由 SplineOptimizer<DIM, SplineType, ...> 决定
    double operator()(const QuinticSplineND<3>& spline,
                      const std::vector<double>& Ts,
                      const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& waypoints,
                      double start_time,
                      const BoundaryConditions<3>& bc,
                      QuinticSplineND<3>::Gradients& grads) const;
};
```

## AuxiliaryStateMap 协议

```cpp
struct AuxiliaryStateMapProtocol
{
    // 辅助优化变量块的维度
    int getDimension() const;

    // 根据参考状态创建无约束辅助变量的初值
    Eigen::VectorXd getInitialValue(const std::vector<double>& ref_times,
                                    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& ref_waypoints,
                                    double ref_start_time,
                                    const BoundaryConditions<3>& ref_bc) const;

    // 在样条更新之前,把辅助变量施加到工作状态上
    void apply(const Eigen::VectorXd& z,
               std::vector<double>& times,
               Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& waypoints,
               double& start_time,
               BoundaryConditions<3>& bc) const;

    // 把工作状态的梯度反向传播到 z
    // 具体样条类型由 SplineOptimizer<DIM, SplineType, ...> 决定
    double backward(const Eigen::VectorXd& z,
                    const QuinticSplineND<3>& spline,
                    const std::vector<double>& times,
                    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& waypoints,
                    double start_time,
                    const BoundaryConditions<3>& bc,
                    QuinticSplineND<3>::Gradients& grads,
                    Eigen::VectorXd& grad_z) const;
};
```

## 运行时集成说明

`SplineOptimizer` 不再持有内部的"已准备问题"或运行时状态。调用方应为每个
活跃的求值流程提供一个 `OptimizationContext`:

```cpp
using Optimizer = SplineTrajectory::SplineOptimizer<3>;

Optimizer optimizer;
Optimizer::OptimizationContext context;

auto status = optimizer.prepareContext(problem, context);
if (!status)
{
    std::cerr << status.message << std::endl;
}

auto spec = Optimizer::makeEvaluateSpec(time_cost, integral_cost);
auto x = optimizer.generateInitialGuess(context);
auto result = optimizer.evaluate(context, x, grad, spec);
if (!result)
{
    std::cerr << result.message << std::endl;
}
```

`prepareContext(...)` 会快照后续求值所需的、依赖配置的映射器与标量状态,
包括:

- `TimeMap`
- `SpatialMap`
- `AuxiliaryStateMap`
- `rho_energy`
- `integral_num_steps`

之后调用 `setConfig(...)` 只会影响**新准备**的上下文,不会修改已准备好的
上下文。

Setup 与校验风格 API 使用 `Status`:

```cpp
Optimizer::OptimizerConfig config;
config.rho_energy = 0.05;
config.integral_num_steps = 32;
optimizer.setConfig(config);

Optimizer::ProblemDefinition problem;
problem.time_segments = durations;
problem.waypoints = waypoints;
problem.start_time = 0.0;
problem.bc = bc;

Optimizer::OptimizationContext context;
auto status = optimizer.prepareContext(problem, context);
if (!status)
{
    std::cerr << static_cast<int>(status.code) << std::endl;
    std::cerr << status.message << std::endl;
}
```

如果只想针对当前问题使用显式的掩码(mask),在准备上下文之前把它直接挂到
问题定义上即可:

```cpp
Optimizer::OptimizationMask mask;
mask.time.assign(durations.size(), static_cast<uint8_t>(1));
mask.waypoints.assign(durations.size() + 1, static_cast<uint8_t>(1));
mask.waypoints.front() = static_cast<uint8_t>(0);
mask.waypoints.back() = static_cast<uint8_t>(0);

problem.mask = mask;

auto status = optimizer.prepareContext(problem, context);
```

如果上游管线提供的是绝对时间点而不是时间段(duration),可以用辅助函数构建
问题,并仍然使用同一个 `prepareContext(...)` 入口:

```cpp
std::vector<double> time_points = {0.0, 1.2, 2.7, 4.0};
auto problem = Optimizer::makeProblemFromTimePoints(time_points, waypoints, bc);
auto status = optimizer.prepareContext(problem, context);
```

求值使用 `EvaluationResult`,错误码模式相同:

```cpp
auto result = optimizer.evaluate(context, x, grad, spec);
if (!result)
{
    std::cerr << static_cast<int>(result.code) << std::endl;
    std::cerr << result.message << std::endl;
}
```

LBFGS 风格的内层循环可以在 `prepareContext(...)` 成功之后使用
`evaluatePrepared(...)`。它跳过 `evaluate(...)` 中重复的防御性解码;
因此上下文与"决策向量有限"的前置条件由调用方负责。外部优化器返回后,
调用 `synchronizeWorkingState(context, x)` 只重建输出样条,而不用再付一次
完整的代价/梯度积分。

可选代价与自定义执行器可以流畅地附加:

```cpp
auto spec = Optimizer::makeEvaluateSpec(time_cost, integral_cost)
                .withWaypointsCost(waypoints_cost)
                .withSampleCost(sample_cost)
                .withTrajectoryCost(trajectory_cost)
                .withExecutor(OpenMPExecutor{});
```

使用 `OpenMPExecutor` 时,被借用的 `integral_cost` 对象必须是**线程安全**
的,因为同一个函数对象实例会被并发地跨段调用。
