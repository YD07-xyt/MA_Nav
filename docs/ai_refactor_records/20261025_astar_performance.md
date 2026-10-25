# A* 性能优化 改造记录

## User Intent

A* 同环境耗时 50–60ms(JPS <1ms),要求按讨论方案优化耗时,保持 `original_astar_search` 接口不变。

## Scope

- src/planner/path_planning/include/path_planning/search/astar.h(私有成员)
- src/planner/path_planning/src/search/astar.cpp(内部实现)

## Out of Scope

JPS、PathPlanningImpl、GridMap、参数与默认值、set_map 整图拷贝(未改)。

## Explorer Findings

### Files inspected

astar.h/.cpp、jps.h/.cpp、path_planning.cpp、grid_map.hpp/.cpp。

### Active logic path

`PathPlanningImpl::path_planning` → `use_jps_ ? jps_search : original_astar_search`。

### Risk notes

原实现耗时主因:`std::string` 哈希键、每次 push `new Node`、无 closed set、每邻居 ESDF 插值无缓存、浮点坐标累加、欧氏启发、每轮取时钟、地图外视为自由。

### Recommended modification boundary

仅 AStar 类内部。

## Modifier Changes

### Files changed

astar.h、astar.cpp。

### Key changes

- 栅格线性地址 + 扁平数组(g/parent/open_gen/closed_gen)+ 代际计数,免分配免清零。
- 堆存 `(f, h, idx)`,惰性删除,f 相同时 h 小优先。
- 逐栅格占用缓存(栅格中心 ESDF),与 JPS 一致;`set_map`/`set_safe_threshold` 时失效。
- 八叉距离启发;超时每 256 次迭代检查一次。

### Behavior preserved

- 接口签名、默认 timeout、起终点碰撞检查(地图外视为自由)、失败返回空 vector。
- 路径首点为精确起点。

### Behavior intentionally adjusted

- 搜索在栅格上进行,中间点为栅格中心(原为起点 + 整数步长偏移);末点改为精确终点(原为距终点 < 1 个分辨率的点)。
- 越界栅格视为障碍,搜索不再走出地图;起终点在地图外时被 clamp 到边界栅格。
- 碰撞在栅格中心求值,而非任意浮点位置。
- 终点栅格允许进入,即使中心在膨胀区内。

### Notes

起点栅格中心被占用时,A* 仍会从该栅格扩展(与原实现一致,不做 JPS 式吸附)。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查(无残留 `AStar::Node` / `position_to_key` 引用)
- [ ] XML / launch / yaml 检查(不涉及)
- [ ] 用户允许范围内的测试或静态检查
- [ ] 如需构建,已取得用户明确许可(未构建)

### Issues found

无静态问题;耗时与路径质量需用户构建后实测。

### Final result

PASS(待编译与实测确认)
