# A* vs JPS 图片地图对比测试 改造记录

## User Intent

用 images/ 下两张黑白图(黑 = 障碍)构建 GridMap,对比 A* 与 JPS 性能,输出轨迹叠加图、Google Benchmark 结果和对比数据图到 images/output/。

## Scope

src/test/include/cv2map.hpp、src/test/src/test_astar_vs_jps.cpp、src/test/CMakeLists.txt。

## Out of Scope

规划器、地图库、比赛参数、根 CMakeLists(`add_subdirectory(src/test)` 为用户已有改动)。

## Explorer Findings

### Files inspected

grid_map.hpp/.cpp、astar.h/.cpp、jps.h/.cpp、根与 src/test CMakeLists、logger.hpp。

### Active logic path

测试程序独立可执行文件,不进入 ma_nav 运行链路。

### Risk notes

- 图片 560x300,按 0.05 m/px 对应 28 m x 15 m 场地。
- 系统已装 libbenchmark-dev 1.6.1,OpenCV 4.5.4。

### Recommended modification boundary

仅 src/test。

## Modifier Changes

### Files changed

- cv2map.hpp:`test_utils::Cv2Map`,读灰度图 → GridMap(原点左下,x 向右,y 向上,灰度 < 128 为障碍,可选膨胀),提供像素/世界坐标互转。
- test_astar_vs_jps.cpp:固定种子采样 20 组可达起终点;对比运行(5 次中位数)+ 输出图片/CSV;再注册 Google Benchmark 用例。
- CMakeLists.txt:增加 OpenCV;去掉 benchmark_main(自定义 main)。

### Behavior preserved

不影响 ma_nav 主程序。

### Behavior intentionally adjusted

无。

### Notes

`kSafeThreshold` 运行时为 0.05 m(用户在编写过程中修改)。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [ ] XML / launch / yaml 检查(不涉及)
- [x] 运行测试程序,检查输出文件与 CSV
- [x] 如需构建,已取得用户明确许可(`colcon build --packages-select ma_nav`,通过,无本次新增代码的 warning)

### Issues found

无。两算法 40 组查询路径长度完全一致(最大差 0.0 m)。

### Final result

PASS
