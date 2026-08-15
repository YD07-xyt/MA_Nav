/**
@goal:
path_planning::path_planning(start, goal)          (JPS/A* + 后处理)
        │  PathPostProcessing::Trajectory { optimized_path, timed_trajectory,
        │                                    start_state, total_time, ... }
        ▼
桥接:Trajectory ──► ProblemDefinition (航点/时间/边界条件)
        ▼
SplineOptimizer<3>: prepareContext → generateInitialGuess
        ▼
LBFGS 迭代:evaluatePrepared(ctx, x, grad, spec)   ← 仓库已有 utils/lbfgs.hpp [blocked]
        ▼
synchronizeWorkingState(ctx, x)
        ▼
采样 ctx.runtime.state.spline ──► 回填 Trajectory(格式不变,下游 MPC/可视化零改动)
*/