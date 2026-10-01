# SplineTrajectory 库升级对比 & 硬约束避障凸走廊方案 研讨记录

- 日期：2026-09-28
- 类型：**调研 / 对比 / 方案分析记录**（本次未修改任何源码文件）
- 范围：`include/planner/traj_optimize/`、`3rd/SplineTrajectory/`、`src/planner/traj_optimize/ma_spline_opt/`、`src/map/grid_map.cpp`、`include/planner/traj_optimize/esdf_Interface.hpp`
- 关联上游文档：`3rd/SplineTrajectory/docs/api_migration.md`、`docs/optimizer.md`、`docs/convex_hull.md`、`docs/numerical_notes.md`、`docs/benchmarks.md`

> 本文件是**事后记录**，不是 writingplans，也不是修改前计划。本文件对应的两次对话均未产生代码改动。

---

## User Intent

两轮对话的原始诉求：

1. **第一轮**：SplineTrajectory 库更新了，新库文件已放在 `include/planner/traj_optimize/spline_opt/`。要求对比 `include/planner/traj_optimize/ma_spline_opt/`（现在在用）、`include/planner/traj_optimize/spline_opt/`（新）、`3rd/SplineTrajectory/`（上游 checkout），**先说明更新了什么、在现有优化算法上有什么算法层面的提升**，"先告诉我对比"（即本轮不要改代码）。
2. **第二轮**：既然 ESDF 无法作为硬约束，追问**如何实现硬约束避障**。
3. **第三轮**：把上面两次对话整理成文档。

关键约束（用户与 AGENTS.md 共同施加）：
- 本轮只做对比与方案分析，不改代码。
- 禁止未经许可的编译/构建。
- 禁止任何 git 提交类操作。
- 不破坏比赛验证过的主逻辑。

---

## Scope

本文档内容覆盖：
- 新旧 SplineTrajectory 头文件的来源、版本、差异事实。
- 库升级带来的 API 破坏面与迁移清单。
- 库升级带来的算法/性能变化（区分"已由源码证实"与"上游声明"）。
- ESDF 不能作为硬约束的原因分析。
- 硬约束避障的可行路径、三种集成档次、凸走廊构建方案对比。
- 当前代码中发现的三个与安全语义相关的漏洞。

## Out of Scope

- 未实际执行任何代码修改。
- 未执行任何编译/构建，因此所有编译期结论均为**静态阅读推断**，未经过编译器验证。
- 未做 OLD vs NEW 的实测性能基准（仓库内不存在这样的基准）。
- 未设计多智能体走廊冲突/让行协议（仅指出这是必须解决的问题）。
- 未涉及云台折叠逻辑本身（只在约束解耦意义上提及 yaw 不应混入避障走廊）。

---

## Explorer Findings

### Files inspected

**旧库（OLD，当前编译进 `planner` 目标的版本）**
- `include/planner/traj_optimize/ma_spline_opt/SplineTrajectory/SplineTrajectory.hpp`（3700 行，md5 `8d14cb7533b2412d772139a9687e146e`）
- `include/planner/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp`（2586 行，md5 `dd144bbeff5532ccaeb32bece5866a94`）

**新库（NEW，用户放入的库文件）**
- `include/planner/traj_optimize/spline_opt/SplineTrajectory/SplineTrajectory.hpp`（3976 行，md5 `f70fbb1f0421703e88ae9496d0733855`）
- `include/planner/traj_optimize/spline_opt/SplineTrajectory/SplineOptimizer.hpp`（1265 行，md5 `baad00b574a55956e617ea0bf5183321`）
- `include/planner/traj_optimize/spline_opt/SplineTrajectory/SplineConvexHull.hpp`（872 行，md5 `460ee0a7028e479e91863c6335cdbd44`）— **全新文件，OLD 无对应物**
- `include/planner/traj_optimize/spline_opt/tarj_opt.h`（9 行占位，文件名拼写为 `tarj_opt`）

**上游 checkout**：`3rd/SplineTrajectory/`（嵌套 git 仓库，线性历史 140 提交，HEAD `126525e`，含 `docs/`、`examples/`、`benchmarks/`、`tests/`）

**现网调用方**
- `include/planner/traj_optimize/ma_spline_opt/traj_optimizer.h`（119 行）
- `include/planner/traj_optimize/ma_spline_opt/cost.hpp`（228 行）
- `include/planner/traj_optimize/ma_spline_opt/optimizer_config.h`（155 行）
- `src/planner/traj_optimize/ma_spline_opt/traj_optimizer.cpp`（365 行）
- `include/planner/controller/traj_interface.hpp`
- `include/ros2/misc/visualizer.hpp`
- `src/planner/replan/fsm_replanner.cpp`
- `include/planner/traj_optimize/esdf_Interface.hpp`
- `include/planner/traj_optimize/minco_opt/grid_map_esdf.hpp`
- `src/map/grid_map.cpp`
- `include/planner/path_planning/post_processing.h`

### Active logic path

```
JPS 路径规划
  → PathPostProcessing（梯形速度时间重采样）→ Trajectory{ raw_path, optimized_path, timed_trajectory, time_segments, total_time }
  → ma_spline_opt::from_path_planning_trajectory()        （optimizer_config.h:108）
  → MaSplineTrajectoryOptimizer::optimize()               （traj_optimizer.cpp:53）
      ├─ OMNI_XY          → optimize_xy()                 （traj_optimizer.cpp:87）
      └─ OMNI_XY_YAW_JOINT→ optimize_xy_yaw_joint()        （traj_optimizer.cpp:300+）
           └─ 每个 stage：optimize_2d_stage / optimize_3d_stage
                setConfig → prepareContext → generateInitialGuess
                → lbfgs::lbfgs_optimize(cost_callback → evaluatePrepared)
           └─ synchronizeWorkingState → getWorkingSpline
  → MAsplineOutput{ trajectory: QuinticSplineND<3>(x,y,yaw), ... }
  → traj_interface.hpp → controller
```

- 权重与约束来源：`MaSplineOptimizerConfig`（`optimizer_config.h:40-58`），两阶段 `stage1/stage2` 独立配置。
- ESDF 注入：`fsm_replanner.cpp:92-93` 与 `:150-151` 构造 `minco_opt::GridMapESDF` 并 `ma_opt_.set_esdf_interface()`。
- 积分采样：`config_.integral_num_steps = 8`（每段 9 个求积点，梯形权重，端点半权）。
- 碰撞兜底：`MaSplineTrajectoryOptimizer::check_trajectory_collision()`（`traj_optimizer.cpp:32-50`），按 0.05s 步长重采样。

### Data flow（关键结论）

- 优化变量分块顺序（旧新库一致）：`[活动时长 T] → [活动 waypoint] → [起末 v/a/j] → [auxiliary]`。
- 时间参数化：`QuadInvTimeMap`，`x_block = toTau(T)`，回传 `backward(tau, T, gradT)`。**新旧库公式逐项相同**。
- 求积与梯度：梯形权重、端点半权、显式时间梯度反向前缀和。**新旧库算术相同**。
- 时间来源：`post_processing.cpp:251-266` 用 `total_time / time_resolution` 平均分段 → `traj.time_segments`。这是**参考初值**，不是约束；`problem.durations` 取相邻 `time_points` 差值即为正确迁移。
- ESDF 数值来源：`grid_map.cpp` 的 `esdf_buffer_`，通过双线性插值提供 `getDistance` / `getDistanceAndGradient`。

### Risk notes

1. **NEW 目前是死代码**：`git status` 显示 `include/planner/traj_optimize/spline_opt/` 整个目录 untracked，且全仓库没有任何文件 include 它（`build/CMakeFiles/planner.dir/DependInfo.cmake` 只编译 `ma_spline_opt/traj_optimizer.cpp`）。
2. **`install/` 里装的仍是 OLD**：`install/ma_nav/include/ma_nav/planner/traj_optimize/ma_spline_opt/SplineTrajectory/SplineTrajectory.hpp` 的 md5 为 `8d14cb75…`，与 OLD 相同。只改源码不同步 `install` 会导致"改了但没生效"。
3. **`SplineConvexHull.hpp` 与 OLD 不兼容**：它调用 `isValid()` / `numSegments()` / `coefficientCount()` / `breakpoints()` / `coefficients()` 以及 `PPolyND<DIM, MaxCoefficients>` 的新模板参数名。想用凸包必须整体迁移。
4. **`SplineConvexHull.hpp` 与 `SplineOptimizer.hpp` 零耦合**：两个新头文件内 grep 不到任何 Hull 符号，必须自行 include 并自行接入。
5. **失效语义变更影响收敛行为**：OLD 在候选点非法时返回 `cost = 0.0` + 零梯度，NEW 返回 `+infinity`。这会改变 L-BFGS 线搜索行为，`params.delta` / `min_step` 需重新审视。
6. **上游自身声明**：固定候选点梯度相对误差 < 3e-13，但独立求解不总得到同一条轨迹。迁移后即使权重不变，轨迹形状也会有漂移。
7. **禁止 fast-math**：NEW 的正确性依赖 `allFinite` / `isValid` 检查与返回 `infinity`。当前 `CMakeLists.txt` 未开启 fast-math，安全，但后续不得引入。
8. **`3rd/SplineTrajectory` 不参与构建**：仓库 `CMakeLists.txt` 未引用 `3rd/`，头文件是靠 `include/` 下的两份拷贝消费的。上游 checkout 只是参考与测试来源。

### Recommended modification boundary

- 本轮：只读调研，不修改。
- 后续若迁移：改动面应限制在 `include/planner/traj_optimize/ma_spline_opt/*` + `src/planner/traj_optimize/ma_spline_opt/*` + `include/planner/controller/traj_interface.hpp` + `include/ros2/misc/visualizer.hpp`，**不改 topic / frame / blackboard key / launch / 参数默认值**。

---

## 第一部分：SplineTrajectory 库更新对比

### 1. 版本溯源（md5 核实，非推断）

| | 位置 | md5 | 上游身份 | 行数 |
|---|---|---|---|---|
| **OLD** `SplineTrajectory.hpp` | `ma_spline_opt/SplineTrajectory/` | `8d14cb7533b2412d772139a9687e146e` | = 上游 `5dde128`（2026-07-15） | 3700 |
| **OLD** `SplineOptimizer.hpp` | 同上 | `dd144bbeff5532ccaeb32bece5866a94` | = 上游 `5dde128` | 2586 |
| **NEW** `SplineTrajectory.hpp` | `spline_opt/SplineTrajectory/` | `f70fbb1f0421703e88ae9496d0733855` | = 上游 HEAD `126525e`（2026-09-12） | 3976 |
| **NEW** `SplineOptimizer.hpp` | 同上 | `baad00b574a55956e617ea0bf5183321` | = 上游 HEAD `126525e` | 1265 |
| **NEW** `SplineConvexHull.hpp` | 同上 | `460ee0a7028e479e91863c6335cdbd44` | = 上游 HEAD | 872 |
| **install 副本** | `install/ma_nav/include/ma_nav/...` | `8d14cb75…` | **仍是 OLD** | 3700 |

结论：`spline_opt/` 三个文件与 `3rd/SplineTrajectory/include/` **逐字节相同**；OLD 与 NEW 是**同一条上游线性历史的两个点**，不是分叉。

### 2. 中间提交（12 个，只有一个破坏性）

```
5dde128  = OLD
39fa5bf  feat: add differentiable convex-hull spline bases          ← 新能力
e0e17a8  perf: implement reusable convex hull optimization kernels  ← 新能力
d504135  feat(spline): integrate convex hulls and structured opt    ← 新能力
ea1af73  refactor(spline)!: unify APIs and own optimizer workspaces ← ★唯一 breaking
54abad9  perf(hull): fuse control conversion and finite-value checks
80616bf  feat(benchmarks): compare frozen planners ...
cf4a56b  refactor: organize tests, examples, and public documentation
956af12  docs: restore README style and add planner comparisons
0a9e987  feat(optimizer): expose read-only physical diagnostics
126525e  = NEW
```

### 3. 三层变化

#### 第 1 层：`SplineTrajectory.hpp` —— 全是改名，数学未动

新增命名空间级类型：
- `BoundaryGradient<DIM>{p,v,a,j}`、`BoundaryGradientPair<DIM>{start,end}`、`SplineGradients<DIM>{inner_points, durations, start, end}`
- `detail::durationsFromKnots()`、`MinDerivativeSplineND<DIM,S>`（2→Cubic / 3→Quintic / 4→Septic）
- `PPolyND::Sampler<N>` + `makeSampler<N>()`、`evaluateDerivatives<N>()`、`evaluateInto()`、`clear()`、`extractInterval()`、`appendRebased()`

删除 / 重命名（需改代码的部分）：

| OLD | NEW |
|---|---|
| `PPolyND<DIM, ORDER>` | `PPolyND<DIM, MaxCoefficients>`（**按位置兼容，按名字不兼容**） |
| `VectorType` / `RowVectorType` / `MatrixType` | `Vector` / `RowVector` / `CoefficientMatrix` |
| `TrajectoryType` | `Polynomial` |
| `isInitialized()` | `isValid()` |
| `getStartTime/getEndTime/getDuration` | `startTime/endTime/duration` |
| `getSpacePoints/getTimeSegments/getBoundaryConditions` | `waypoints/durations/boundary` |
| `getTrajectory/getPPoly` | `polynomial()` |
| `getTrajectoryCopy/getPPolyCopy` | `copyPolynomial()` |
| `getNumSegments/getNumPoints/getDimension` | `numSegments/numPoints/dimension` |
| `getCumulativeTimes()` | `breakpoints()` |
| `getBreakpoints/getCoefficients` | `breakpoints/coefficients` |
| `getEnergy/getEnergyGrad/getEnergyGrad*` | `energy/energyGradient/energyDurationGradient` 等 |
| `propagateGrad()` | `backward()` |
| `Gradients::times` | `Gradients::durations` |
| `BoundaryStateGrads / BoundaryDualGrads` | `BoundaryGradient / BoundaryGradients` |
| `ORDER / COEFF_NUM` | `kDegree / kCoefficientCount`（**数值相同**：quintic = 5/6） |
| `Segment::evaluate()` / `getCoeffs()` | `evaluateLocal()` / `coefficients()` |
| `evaluate(t, int* hint, order)` | **删除**（改用 `makeSampler<N>()`） |

行为变化：
- OLD 的 mutable 派生系数缓存被删除 → OLD 的 `const evaluate()` 会写共享状态（**并发即数据竞争**），NEW 无 mutable、可重入。
- 系数存储从"`coeffs_` + `trajectory_` 两份"改成 **PPoly 单一所有者 + 原地求解**（`solveQuinticInPlace`）。
- 无效样条的 `energy()` 由返回 `0.0` 改为返回 **NaN**；无效阶数求导改为 **throw**。
- `generateTimeSequence` 语义变化：NEW 在 `end_t != start_t` 时精确追加终点，跨步提前 break，非法输入抛异常。

**未变**：三对角/块三对角求解数学、`energy()` 闭式常数（`36T|c3|²+144T²c4·c3+192T³|c4|²+240T³c5·c3+720T⁴c5·c4+720T⁵|c5|²`）、`BoundaryConditions` 字段与三个构造函数、`evaluate(t, order)` 两参签名、`update(durations, points, start_time, bc)`、`computeBasisFunctions`。

#### 第 2 层：`SplineOptimizer.hpp` —— 破坏性重构（`ea1af73`）

| OLD | NEW |
|---|---|
| `SplineOptimizer<DIM, Spline=…, TimeMap, SpatialMap, AuxMap>` | `SplineOptimizer<Spline, Parameterization>`，`DIM = Spline::kDimension` |
| 调用方持有 `OptimizationContext`（prepared + runtime 缓冲） | **类型被删除**，工作区全部私有；不可拷贝、**可移动**；一实例一并发求解 |
| 5 种 cost 类型 + `EvaluateSpec` + `makeEvaluateSpec` + `with*Cost` 链 | 一个普通 struct 的 **6 个可选命名成员** |
| `VoidTimeCost/VoidIntegralCost/VoidWaypointsCost/VoidSampleCost/VoidTrajectoryCost` | `NoObjective` / `NoAuxiliaryMap`；缺省即"不参与" |
| `ProblemDefinition{time_segments, bc}` | `SplineProblem{durations, boundary, waypoints, start_time, mask}` |
| `OptimizerConfig{rho_energy, integral_num_steps, *map 指针}` | `OptimizerOptions{energy_weight, integration_steps, record_samples}` |
| `ErrorCode`（嵌套，含 `NullContext` 等） | `OptimizationError`（命名空间级：`None/NotPrepared/InvalidInput/DimensionMismatch/NumericalFailure/SamplingNotPrepared/GradientMismatch`） |
| `ResultBase{ok, code, std::string message}` | `OptimizationStatus{code, index, const char* message}`，`explicit operator bool` |
| `EvaluationResult{cost = 0.0}` | `EvaluationResult{cost = +infinity}` |
| `setConfig(cfg)` → `prepareContext(problem, ctx)` | `prepare(problem, options)` |
| `generateInitialGuess(ctx)` | `initialGuess()`（未 prepare 抛 `std::logic_error`） |
| `evaluatePrepared(ctx, x, g, spec)` | `costAndGradient(x, g, objective)` |
| `evaluate(ctx, x, g, spec)` | `evaluate(x, g, objective, executor)` |
| `synchronizeWorkingState(ctx, x)` | `updateAccepted(x)` |
| `getWorkingSpline(ctx)` | `polynomial()` / `copyPolynomial()` |
| `getDimension(ctx)` | `dimension()` |
| `makeProblemFromTimePoints(...)` | **删除**，自行构造 `SplineProblem` |
| `makeFullOptimizationMask(n)` | **删除**，无替代 helper |
| `checkValidity` / `encodeWorkingState` / `getActiveConfig` | **删除** |
| `setRecordIntegralSamples(enable, ctx)` | `OptimizerOptions::record_samples`（prepare 期固定） |
| 成员 `checkGradients(ctx, x, spec, eps, tol)` | 自由函数 `checkGradients(optimizer, x, objective, step, tol)` |
| `SpatialMap::getUnconstrainedDim` / `backwardGrad()` 返回向量 | `dimension()` / `backwardInto(..., Ref<VectorXd>)`（**覆写**目标切片） |

**积分代价回调协议变化（最关键的一处）**

```cpp
// OLD：12 个位置参数
double operator()(const IntegralPointInfo& point,
                  const Vec& p, const Vec& v, const Vec& a, const Vec& j, const Vec& s,
                  Vec& gp, Vec& gv, Vec& ga, Vec& gj, Vec& gs, double& gt) const;

// NEW：3 个聚合参数
double operator()(const SplineTrajectory::IntegralPointInfo& point,
                  const SplineTrajectory::SampleState<DIM>& state,      // .p .v .a .j .s
                  SplineTrajectory::SampleGradient<DIM>& gradient) const; // .p .v .a .j .s .time
```

- `gt` → `SampleGradient::time`，**算术完全不变**（`local_acc_gdT += gt * alpha * common_weight`、`local_acc_explicit_time_grad += gt * common_weight` 逐字保留）。
- `SampleGradient` 每个采样点前清零，输出是**累加**语义（不要 `=` 覆盖，不要自己再乘梯形权重）。

**新增的 6 个命名阶段与签名**

| 成员 | 签名 | 含义 |
|---|---|---|
| `duration` | `(const std::vector<double>&, Eigen::Ref<Eigen::VectorXd>)` | 物理时长代价与偏导 |
| `integral` | `(const IntegralPointInfo&, const SampleState<D>&, SampleGradient<D>&)` | 无权重被积函数与局部偏导 |
| `sample` | `(const Opt::SampleBuffer&, Eigen::Ref<Opt::SampleGradMatrix>, Eigen::Ref<Eigen::VectorXd>)` | 跨采样点代价 + 显式全局时间偏导 |
| `coefficient` | `(const ParameterView<S>&, CoefficientGradient<S>&)` | 多项式系数与独立时长偏导 |
| `parameter` | `(const ParameterView<S>&, ParameterGradient<S>&)` | 样条伴随后的物理参数偏导（`addWaypoint(global_index, partial)`） |
| `decision` | `(const DecisionView&, Eigen::Ref<Eigen::VectorXd>)` | 映射域直接代价（用 `+=`） |

**未变**（重要）：决策向量分块顺序、梯形权重与端点半权、显式时间梯度反向前缀和、掩码默认语义（空 `waypoints` 掩码 = 只优化内部点）、`QuadInvTimeMap` 公式、`IntegralPointInfo` 字段、`beginEvaluation()` 钩子。

#### 第 3 层：`SplineConvexHull.hpp` —— 全新，872 行

公开 API：
- `enum class ConvexHullBasis { Bezier, MINVO }`
- `struct HullOptions { basis; derivative_order; subdivision_depth; }`（**无 tolerance/epsilon 字段**）
- `struct ConvexHullKernel`（维度无关的拓扑数据 + `memoryBytes()`，在全局 `mutex` 保护的 `weak_ptr` 缓存里共享）
- `ConvexHullWorkspace<DIM>`：`fromPPoly / configure(两种重载) / update / matches / isConfigured / isValid / controls() / pieces() / pieceInfo(i) / pieceControls(i) / backward(两种) / backwardAdd / backwardPieceTimesAdd / powerToControlMatrix`
- 自由函数 `toBezier(poly, derivative_order, subdivision_depth)`、`toMINVO(...)`
- `PieceInfo{source_segment, subdivision_index, source_fraction_begin/end, start_time, duration}` — 把细分叶映射回源段

数学：
- Bezier：`matrix(i,k) = C(i,k)/C(degree,k)`，标准 power→Bernstein 映射，**曲线不变**。
- MINVO：硬编码 degree 0–7 精确矩阵（来源 Tordesillas, MIT ACL, BSD-3），超出抛 `invalid_argument`。
- 导数阶 `r`：`derivative_factors(k) = fallingFactorial(k+r, r)`，`degree = source_num_coeffs - r - 1`。
- 细分：`leaves = 2^depth`，每叶 `u ∈ [a, a+h]`，限制矩阵 `restriction(j,k) = C(k,j)·a^(k-j)·h^j`，`long double` 累加。
- 控制点含 `T^power` 因子，即**物理秒**单位。

复杂度（N 段，r 阶，d 深度，`cp = 系数数 - r`）：`update` 与 `backwardAdd` 均 `O(N·2^d·cp²·DIM)`；`backwardPieceTimesAdd` `O(N·2^d)`；`configure` O(1) + 取核（含一次全局锁）。

**与优化器的关系：零耦合。** `SplineOptimizer.hpp` 里 grep 不到任何 Hull 符号，必须自行 include 并按 `coefficient` 阶段接入（上游范例 `examples/convex_hull_optimizer.cpp`、`tests/optimizer/test_optimizer.cpp:288` 的 `workspace.backwardAdd(control_gradient, gradient.coefficients, gradient.durations)`）。

### 4. 算法 / 性能层面的实际提升

按对本项目的收益排序：

**① 归一化求积基缓存（最大热路径收益，迁移即白拿）**
- OLD：每个 L-BFGS 回调、每段、每采样点都重算物理时间 `t` 的幂。本项目 `integral_num_steps = 8` → 9 组/段，200 次迭代 × 2 阶段，全部重复计算。
- NEW：`prepare()` 里按 `α = k/K ∈ [0,1]` **只算一次**（`integration_steps + 1` 组），评估时改为"每段一次 `T^power` 系数缩放 + 乘 `1/T^n`"，200 次迭代反复复用。
- 副产品：`T` 的幂改用连乘而非 `std::pow`；参数留在 [0,1]，**比"物理 t 的 5 次方"条件数更好**（数学等价，`T^n·c_n·α^n = c_n·t^n`）。

**② `kDerivativeOrder` 剪枝（每行代码收益最大）**
- 新协议要求积分代价声明 `static constexpr int kDerivativeOrder`（0–4，默认 4）。
- 本项目 `RobotIntegralCost` 只写 `gp/gv/ga` → 设为 `2`，可让优化器用 `if constexpr` **直接编译掉** snap/crackle 状态、它们的梯度外积、两个漂移点积。OLD 无论是否需要都算满 5 阶。
- 语义：声明 `d` 表示"最多写第 d 阶梯度"，优化器额外算 `d+1` 阶用于时间漂移。
- **陷阱**：设小了不报错，梯度被静默丢弃（分支不实例化）。上游 `Penalties.hpp:44` / `esdf_optimizer.cpp:20` 用 `2`。

**③ 凸包硬约束 —— 唯一的算法级新能力**
- 保证形式："一段曲线必然落在其控制点凸包内"。
- 用途一（**合法且充分**）：位置/速度/加速度控制点全部满足**凸走廊半空间** ⇒ 整段在走廊内 ⇒ 整段无碰。
- 用途二（**无任何非凸障碍**）：速度/加速度/角速度控制点的范数上界可升级为**整段硬保证**，替代现在"仅 9 个求积点上的软惩罚"。
- **边界**：ESDF 自由空间一般非凸，**控制点处 ESDF 为正不能证明整段无碰**。凸包不能直接用于 ESDF，必须先有凸走廊（见第二部分）。
- `subdivision_depth` 提供单调的精度/开销旋钮；`pieceInfo(i).source_segment` 支持按源段回溯。

**④ 失效语义修正（会改变收敛行为，必须知晓）**
- OLD：候选点非法（如时长解成非正）返回 `cost = 0.0` + 零梯度；NDEBUG 下那个 `assert(false)` 被编译掉 → **L-BFGS 会把不可行点当成"零代价最优"接受**。
- NEW：返回 `+infinity`。
- 影响：线搜索行为变化，`params.delta`（5e-3）与 `min_step`（1e-32）需重新审视。

**⑤ 评估期零分配**
- OLD 每次回调 `integral_samples.resize/clear`、`sample_position_grad_buffer.resize`、并构造 `std::vector<std::string>` 报错信息。
- NEW 只在 `prepare()` 分配一次；文档明确"Fixed-topology library evaluation does not allocate"。
- 对本项目 `fsm_replanner` 的实时循环直接消除分配器抖动。

**⑥ 初始猜测逆映射只解一次**
- `initialGuess()` 现返回 `prepare()` 期算好的拷贝（提交 `126525e` 理由即"逆映射自身可能跑 L-BFGS"）；OLD 每次调用都重跑 `toUnconstrained`。
- 附带：初值候选在 `prepare()` 期即被构建与校验，坏参考状态在 prepare 就失败，而非首次 evaluate。

**⑦ `evaluate()` 不再重复解码候选点**
- OLD 的 `evaluate()` 先 `validateEvaluateSpec`（建临时 `WorkingState`、解码、apply、校验），再 `runEvaluation` 重做一遍；NEW 每次评估只有一次 `buildCandidate`。

**⑧ 其他**
- 删除 OLD 的 1 ms 参考时长下限 `MIN_VALID_DURATION`（OLD:1040/2262）。现在 0.5 ms 段会被接受，但上游 `numerical_notes.md` 警告极短段条件数差，并给出 0.017 s 的未解决案例（相对梯度差 `2.80e-4` 与 `1.16e3`）。
- `checkGradients` 变为自由函数，可对本项目加权 ESDF+yaw 代价做单测。
- 新增只读诊断 `parameters()`（`{polynomial, durations, waypoints, boundary, start_time}`）与 `energy()`，替代伸手进 `ctx.runtime.state.*`。
- `updateAccepted(x)` 成为显式信任边界（上游明确："求解器返回的最后一个试探点可能不是被接受的解"）。

**⚠️ 关于性能数字的重要澄清**
上游 `docs/benchmarks.md` 的表格（ESDF 12 段：评估 33.4→26.1 µs，求解 5.07→4.05 ms）比较对象是 **NEW vs 原生 MINCO / SUPER**，**不是 NEW vs OLD**。仓库内不存在 OLD-vs-NEW 基准，**不能把"快 20%"直接算到本项目头上**。且上游声明"独立求解不总能得到同一条轨迹"，迁移后轨迹形状会漂移，需重新验收。

### 5. 迁移清单（本轮未执行）

**Layer A：`SplineTrajectory.hpp` 改名（机械）**

涉及 `traj_optimizer.cpp`（`:10, :12, :17, :25, :27, :37, :39, :41, :143, :361`）、`traj_interface.hpp`（`:17, :22, :29, :31, :101, :102`）、`visualizer.hpp`（`:88, :98, :214, :215, :240-242, :245, :248, :460`），约 20 处。`evaluate(t, order)` 与 `update(...)` 不变。

**Layer B：优化器 API（破坏面集中在 4 处类型/签名）**

| 现网代码 | 迁移 |
|---|---|
| `using Opt2D = SplineOptimizer<2>;`（`traj_optimizer.h:21`） | `SplineOptimizer<QuinticSplineND<2>>` |
| `Opt2D::OptimizationContext* ctx`（`:57,:85`）+ 成员 `ctx_2d_/ctx_3d_`（`:113,:116`） | 删除，工作区归优化器所有 |
| `decltype(Opt2D::makeEvaluateSpec(...))`（`:59-62,:87-90`） | 删除，代价 struct 即类型 |
| `evaluatePrepared(*ctx, x, g, *spec)`（`:67,:94`） | `costAndGradient(x, g, objective)` |
| `Opt2D::ProblemDefinition`（`:70,:97`） | `Opt2D::Problem` |
| `makeProblemFromTimePoints(...)`（`.cpp:117,:340`） | 自行填 `durations[i] = t[i+1]-t[i]`、`start_time = t.front()`、`waypoints`、`boundary`、`mask` |
| `OptimizerConfig` + `rho_energy` + `integral_num_steps`（`.cpp:154-156,:215-217`） | `OptimizerOptions` + `energy_weight` + `integration_steps` |
| `setConfig(cfg)`（`.cpp:158,:219`） | 删除，改为 `prepare(problem, options)` 参数 |
| `prepareContext(problem, ctx)`（`.cpp:159,:220`） | `prepare(problem, options)` |
| `generateInitialGuess(ctx)`（`.cpp:164,:225`） | `initialGuess()` |
| `getDimension(ctx)`（`.cpp:165,:226`） | `dimension()` |
| `synchronizeWorkingState(ctx, x)`（`.cpp:133,:353`） | `updateAccepted(x)` |
| `getWorkingSpline(ctx)`（`.cpp:138,:357`） | `parameters()` 重组样条，或保留并 `update()` 自己持有的样条 |
| `ctx_2d_.runtime.state.times / .start_time`（`.cpp:140-141,:358-359`） | `parameters().durations / .start_time` |

**Layer C：代价协议重写（唯一非机械改动）**

```cpp
template <int DIM>
struct RobotIntegralCost {
    static constexpr int kDerivativeOrder = 2;   // 只写 gp/gv/ga
    // 权重字段全部保留
    double operator()(const SplineTrajectory::IntegralPointInfo& /*point*/,
                      const SplineTrajectory::SampleState<DIM>& state,
                      SplineTrajectory::SampleGradient<DIM>& gradient) const
    {
        // 算术体逐字保留；仅把 p/v/a ← state.p/.v/.a，gp/gv/ga ← gradient.p/.v/.a
        // 可删除多余的 gp/gv/ga.setZero()（SampleGradient 每点已清零）
    }
};
// 再加包装
struct Objective2D { TimeCost duration; RobotIntegralCost<2> integral; };
struct Objective3D { TimeCost duration; RobotIntegralCost<3> integral; };
```
`TimeCost` 本身无需改动（`Eigen::VectorXd&` 可绑定 `Eigen::Ref<Eigen::VectorXd>`）。

**其他必须同步**
- `makeEvaluateSpec` 强制左值检查（`is_lvalue_reference_v`）被移除，改为要求 objective 生命周期覆盖调用。
- `evaluate` 现要求 `gradient.size() == dimension()`，尺寸不符返回 `DimensionMismatch`，**不再 resize**。L-BFGS 会预分配 `g`，故现网安全；手写调用方需自行预分配。
- 迁移后必须同步 `install/`，否则装出来的包仍是 OLD。
- `spline_opt/tarj_opt.h` 是 9 行占位（匿名 namespace + 文件名拼写错误），无人 include；若启用需先删或改名，否则有 ODR/链接隐患。

---

## 第二部分：硬约束避障与凸走廊构建

### 1. 为什么 ESDF 当不了硬约束

硬约束要求约束集 `{d(p) ≥ 0}` 为凸：

1. **几何上非凸**：自由空间的零水平集是障碍物边界，绕柱子必须走一侧 → 约束集天然非凸。
2. **数值上非光滑**：`GridMap::getDistance` 是**双线性插值**（`grid_map.cpp:60-89`），格子边界上梯度不连续 → 任何基于梯度的求解器都会在格子边界处震荡或卡住。
3. **两侧有障碍时会卡死**：两侧障碍时梯度把点往中间推，推不动即停，既非可行也非最优。

**凸包证书也解决不了**：它只保证"控制点都在凸集内 ⇒ 整段在凸集内"，即**保证在某个凸集内，不是保证不在障碍物内**。

因此唯一路径：
> 不要约束 `d(p) ≥ 0`，而是**预先用 ESDF 把自由空间挖出一个凸走廊（半空间交）**，再强制整条曲线待在走廊里。走廊内的约束全是线性的，线性约束才是真正的硬约束；"无碰撞"由走廊的构造过程保证。

**关键有利条件**：走廊来源现成——`Trajectory::raw_path` / `optimized_path` 是 `std::vector<Eigen::Vector2d>`（`post_processing.h:45-46`），而 JPS 路径是无碰撞的。**不需要全局凸分解**。

### 2. 走廊构建方案总览

| 方案 | 建走廊耗时 | 保守性 | 实现复杂度 | 依赖 | 在线可行 |
|---|---|---|---|---|---|
| 1. ESDF/栅格直接膨胀 | ~0.05 ms（M=200） | 高 | 极低 | 现有 ESDF 接口 | ✅ 每周期 |
| 2. IRIS / 迭代膨胀 | 1–50 ms / 区域 | 最低 | 高（SOCP + NN） | 求解器 + KD-tree | ⚠️ 低频/离线 |
| 3. 球体覆盖 Seed（DecompUtil 式） | 0.1–1 ms | 中高 | 中 | ESDF 或占栅 + 二分 | ✅ 每周期 |
| 4. 全局凸分解 | 离线重 / 查询 O(1) | 中（需重叠） | 中高 | 地图预处理 | 查询✅ 分解❌ |
| 5. 走廊增长（Fast-Planner 式） | 0.2–2 ms | 中 | 中 | ESDF | ✅ 每周期 |
| 6. 学习/启发式 | 推理 ~ms | 不可控 | 高 | 训练数据/GPU | ⚠️ 不可审计，不建议 |

核心权衡：**IRIS 的保守性 ↔ 走廊增长法的速度**。比赛中 90% 情况走廊增长法够用，IRIS 留给"窄通道一开始就建不出走廊"的回退路径。

**方案 1（本项目最现实起点）**
```
对路径采样点 q_j:
    d_j = ESDF(q_j),  n_j = -∇ESDF(q_j)          // 指向最近障碍
    半空间:  n_jᵀ p ≥ n_jᵀ q_j + d_j - r_foot
```
- 半径可用球覆盖引理 `R_j = min(d_j, 0.5·|q_{j+1}-q_j|)`，保证段间连续，**无需最近邻搜索，每步 O(1)**。
- 缺点：只贴了障碍的支撑超平面，拐角处严重保守（走廊收成一点）。
- 改进：半径不再只取最近障碍，而是沿垂直方向连续扩张，每次碰障碍就把该处半空间加入约束集（即方案 5 的廉价栅格版）。

**方案 2（IRIS 家族）**
```
种子 p0 → 循环 { 对每条边找最近障碍 → 加切平面 → SOCP 最大化内接椭球 → 内接多面体 } 至收敛
```
- 变体：Fast Iterative Region Inflation、Only-Rounding（只反复加切平面）、稀疏/增量 IRIS。
- 本项目定位：**不要每周期全跑**，只在①初始化 ②走廊失败回退 ③离线预计算已知区域（如低矮隧道区）时调用。

**方案 4 的关键坑**：相邻凸块不重叠时样条跨块不可行，必须做**重叠扩张**。

### 3. 集成档次（接进优化器）

| | 做法 | 硬保证 | 代价 |
|---|---|---|---|
| A 软惩罚照旧 | 现状 | ❌ 有限权重下永远可违反 | 0 |
| B 求积点硬约束 | 在 `IntegralPointInfo` 的 `segment_index/step_index/interiorBoundaryIndex()` 上给每段每个求积点挂该段半空间 | ⚠️ 只在 9 个求积点上成立，点间鼓包无保证 | 小，兼容现有代价结构 |
| C 凸包整段证书 | `toMINVO(polynomial(), 0, depth)` 取控制点，全部满足半空间 ⇒ **整段**在走廊内 | ✅ 整段严格无碰 | 中：需 `ConvexHullWorkspace` + `coefficient` 阶段 + `backwardAdd()` |

**C 才是真正的"硬约束避障"**，前置条件已满足（参考轨迹可行 → 沿参考路径建走廊）。
**B + 事后密检 + 失败回退**是这一代比赛的合理工程折中（上游 `corridor_optimizer.cpp` 自己也是优化完再按每段 200 点采样报 `sampled corridor violation`）。

### 4. 硬约束的数学（2D 全向 + yaw）

```
对某段插值点索引 k、半空间 (n, b)：
    nᵀ P_k ≥ b + r_foot
    ∂/∂P_k = n,   ∂/∂T = 0        ← 常数雅可比
```
- **不需要 ESDF 对时间的导数**——线性约束的好处，`∂p/∂T` 漂移项不参与。
- `ParameterGradient::addWaypoint(global_index, partial)` 专为 waypoint 偏导准备，自动处理首尾端点，越界抛 `out_of_range`。
- yaw 与位置解耦：避障只用 `p.head<2>()`；yaw（云台折叠、朝向）是另一组约束，**不要混进同一条半空间**。

### 5. 实施要点（决定成败）

1. **半空间用隐式表示，不要构造显式多面体**。直接持有 `{(n_i, b_i)}`，约束写 `n_iᵀ P_k ≥ b_i`。GCOPTER 的"多面体地图"本质就是这个（上游 benchmark `PlannerCosts.hpp` 的 `usesPolytopeMap` / `PolytopeH` 即 `H·[p;1] ≤ 0`）。
2. **约束不能只加在 waypoint 上**。必须配合求积点级（便宜但不严格）或凸包整段证书（严格）。
3. **安全距离用"腐蚀地图"实现**：在栅格层对占据栅格做半径 `r_foot + safe_distance` 的形态学腐蚀，ESDF 从腐蚀后的地图算。这样半空间数量与约束数量不变，且所有下游消费者（JPS、事后检查、可视化）自动一致。不要塞进每个半空间的 `b`。
4. **回退链必须显式设计**：
   ```
   走廊增长法 → 失败 → 降 r_foot / 增大采样步长 → 失败
              → IRIS / 离线预分解（若能覆盖）→ 失败
              → 退回软惩罚，保持上一条轨迹，拒绝本次重规划
   ```
   硬约束下"无解"会静默表现为求解器原地不动，必须主动检测 `violation(初值) > 0`。
5. **相邻区域必须叠覆**（区域 i 与 i+1 有非空交），否则样条在接缝处无可行点。最省事做法是"垂直平分面 + 双方各退让一点"。
6. **边界与未知区域当成障碍**（见下面第 6 节）。

### 6. 当前代码中必须先修的安全语义漏洞

这三条是**所有硬约束方案的前提**，且不改变正常情况下的规划行为：

1. **出图被当成安全**
   - `check_trajectory_collision`（`traj_optimizer.cpp:32-50`）：`esdf->isInside()` 为 false 时**不返回 false** → 轨迹跑出地图被判为安全。
   - `GridMap::isCollision`（`grid_map.cpp:149-154`）：`if (!isInMap(pos)) return false;` 同样把出图当安全。
2. **出图处软惩罚失效**
   - `getDistance` 在地图外返回 `std::numeric_limits<double>::max()`（`grid_map.cpp:61`）。
   - `cost.hpp:78` 是 `if (esdf && esdf->isInside(...))` → 出图时**整个避障项被跳过**，优化器没有理由不进地图外。
3. **地图边缘插值被 clamp 污染**
   - `getDistanceAndGradient` 对 4 个邻居索引调 `boundIndex()` 钳制（`grid_map.cpp:117`）→ 地图边缘一格内的梯度不是真实梯度，做硬约束时边缘会"假可行"。

### 7. 推荐落地路线（依赖关系是强制的）

```
① 修出图语义（上述 3 个漏洞）              ← 独立、低风险、行为改动最小，先做
② 沿 JPS raw_path 建走廊（O(M)，纯新增）    ← 不碰现有 SoftESDF
③ 上 B（求积点硬约束）+ 可行性与回退链      ← 复用现有 stage1/stage2
④ 迁移到 NEW API 后升级到 C（凸包整段证书）
   注：SplineConvexHull.hpp 硬依赖 NEW 样条 API，OLD 上塞不进去
⑤ septic/min-snap 时对 velocity/acceleration 控制点做同样的凸包约束
   → v_max / a_max / yaw_rate_max 从软惩罚升级为整段硬保证
   （这一步没有非凸障碍，是纯白拿收益，可先于避障硬约束上线）
```

**不建议**在此阶段上 IRIS 或全局凸分解：它们是"窄通道覆盖率"的解决方案，而当前瓶颈是**根本没有走廊**和**不可行回退未设计**。

---

## Modifier Changes

### Files changed

本次**未修改任何源码**。

新增一个文档文件：

```
docs/ai_refactor_records/20260928_spline_library_upgrade_and_hard_constraint_corridor.md
```
（本文件；`docs/ai_refactor_records/` 目录此前不存在，本次创建）

### Key changes

无代码改动。

### Behavior preserved

线上规划行为 100% 未变：未改任何 `.h/.hpp/.cpp/.yaml/.xml`，`spline_opt/` 仍是死代码且未被 include，`install/` 未动。

### Behavior intentionally adjusted

无。

### Notes

- 本记录同时充当"后续迁移/硬约束实现的输入材料"，包含迁移清单与推荐路线。
- 上游 `3rd/SplineTrajectory/docs/api_migration.md` 标题写 "from d504135"，但 OLD 实际哈希等于 `5dde128`；该表内容对 OLD→NEW 仍然适用，仅标题的基准提交标注已过期。以本记录第 1 节的 md5 溯源为准。

---

## Auditor Review

### Checks performed

- [x] 关键路径检查：`fsm_replanner → ma_spline_opt → traj_interface` 调用链已读通
- [x] diff 检查：新旧头文件逐文件 diff（增量行数已统计），并逐字节 md5 比对
- [x] grep 检查：`makeEvaluateSpec` / `evaluatePrepared` / `OptimizerConfig` / `OptimizerOptions` / `costAndGradient` / `ConvexHull` 等符号在两份头文件与全仓库的分布
- [x] XML / launch / yaml 检查：本次未涉及，但已 grep 确认 `CMakeLists.txt` 未引用 `3rd/`，头文件靠 `include/` 两份拷贝消费
- [x] 用户允许范围内的静态检查：md5sum、git log/show/diff（只读）、`wc -l`、`grep`
- [x] 如需构建，已取得用户明确许可：**未构建**
- [x] 交叉验证：两个独立子代理分别核对 `SplineOptimizer.hpp` 与 `SplineTrajectory.hpp`+`SplineConvexHull.hpp`，结论与主代理独立核对结果一致
- [x] 修正子代理的一处误判：子代理曾把 OLD `SplineOptimizer.hpp` 判为"in-repo fork、非上游"，经主代理按提交逐版本哈希扫描，确认其**等于上游 `5dde128`**，非 fork

### Issues found

均为**待处理问题**，非本次引入：

1. `install/` 中仍是 OLD 头文件（md5 `8d14cb75…`），源码与安装树不一致。
2. `spline_opt/tarj_opt.h` 为 9 行占位，匿名 namespace + 文件名拼写错误，无人 include，启用前需清理。
3. 出图语义漏洞 3 处（见第二部分第 6 节），会导致"走廊 ⊆ 自由空间"的前提失效。
4. `MAsplineOutput::time_segments` 与 `.start_time` 为**死字段**（`.cpp:140-141,:358-359` 写入，全仓库无任何读取点）。
5. 所有编译期结论（尤其是 `TimeCost` 的 `Eigen::VectorXd&` 能否绑定 `Eigen::Ref<Eigen::VectorXd>` 满足 `duration` trait）**未经编译器验证**，属未验证推断。

### Final result

**PASS**（本次交付物为文档，无代码改动，无回归风险）

说明：
- 本记录的重点是**已被源码/md5/git 证实的事实**，以及**明确标注为"未验证推断"或"上游声明"的部分**，两者在正文中已区分。
- 未执行构建：AGENTS.md 禁止在未获用户明确许可前运行构建命令。
- 未执行任何 git 提交类操作，工作区文件由用户自行检查与提交。
