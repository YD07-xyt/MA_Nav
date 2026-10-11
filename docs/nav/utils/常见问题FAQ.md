---
module: utils
doc: 常见问题FAQ
git_rev: 8f98daee5bc7
evidence_digest: be346b5265f1410d
worktree_dirty: false
generated_by: dsh + docs/tools（clangd documentSymbol / python extractors）
generated_at: 2026-10-02T08:00:00Z
status: draft
reviewer: 未审核
---

# utils 模块 常见问题FAQ

> 只收录能在代码里找到依据的问题；属于经验判断的标【推断】。

## Q1：怎么新增一个日志通道？

**做法**：用通道工厂创建（会自动套用统一彩色格式）
（`src/utils/include/utils/logger.hpp:111`），
并把它作为同名的头文件内变量暴露出去，例如已存在的 7 个彩色通道
（`src/utils/include/utils/logger.hpp:119-125`）。

**注意**：通道名在进程内唯一；若外部代码用 spdlog 注册了同名通道会冲突
（`src/utils/include/utils/logger.hpp:112`）。基础通道用的是默认格式，
彩色通道才是「时间 + 通道 + 级别 + 文件行」的格式（`src/utils/include/utils/logger.hpp:114`）。

## Q2：为什么日志只打印到终端，没有落盘？

**原因**：文件日志通道的实现被注释保留了，当前只创建了 stdout 彩色通道
（`src/utils/include/utils/logger.hpp:126-128`）。

**做法**：若要落盘，需要启用注释里的轮转文件通道并把它挂到通道上。

## Q3：怎么测量一段代码的耗时？

**做法**：在作用域里构造一个计时器对象，离开作用域时自动上报
（`src/utils/include/utils/scope_timer.hpp:44`）；
单位会按量级自动在 ns/us/ms/s 之间切换
（`src/utils/include/utils/scope_timer.hpp:62-79`）。
也可用带开关的构造形式或运行时的开关方法临时关闭
（`src/utils/include/utils/scope_timer.hpp:53`、`src/utils/include/utils/scope_timer.hpp:82`）。

**注意**：该 setter 在源码里拼写为 `set_enbale`
（`src/utils/include/utils/scope_timer.hpp:82`），按此拼写调用。

## Q4：参数装载为什么会抛异常？

**两种来源**：

1. 声明了「必填」但 YAML 里没有该键 → 抛 `std::invalid_argument`
   （`src/utils/include/utils/yaml_loader.hpp:100-102`）。
2. **参数文件路径不存在** → YAML::LoadFile 的异常没人捕获，会直接从装载函数抛出去
   （`src/utils/include/utils/yaml_loader.hpp:49`）。

**注意**：普通键缺失或类型不匹配**不会**抛异常，只会回落默认值并打印告警
（`src/utils/include/utils/yaml_loader.hpp:69-74`）。

## Q5：为什么装载参数时文件会被读很多次？

**原因**：装载实现与路径解析实现各自直接调用读文件，没有缓存
（`src/utils/include/utils/yaml_loader.hpp:49`、`src/utils/include/utils/yaml_loader.hpp:126`），
因此装载 N 个参数至少 N 次文件读取；**路径键（含 `/`）会读两次**，即 2N 次，而地图模块的参数全部是路径键。

**做法**：参数很多的模块可以考虑在装载器里缓存 YAML 根节点；
当前行为对正确性无影响（见 [待确认清单](./待确认清单.md)）。

## Q6：为什么下游 include 本模块的头会编译失败？

**原因**：库把 spdlog 与 yaml-cpp 声明为 PRIVATE
（`src/utils/CMakeLists.txt:13`），
但公共头直接 include 它们
（`src/utils/include/utils/logger.hpp:5-8`、`src/utils/include/utils/yaml_loader.hpp:12`）。
下游因此必须自己提供这两个包。

**做法**：要么把它们改成 PUBLIC，要么把这些头的实现细节移入 .cpp
（后者与本模块「纯头文件」的设计冲突）。

## Q7：`plotter.hpp` 和 `matchit.h` 能用吗？

**现状**：

- `tools::Plotter` 是一个通过 UDP 把 JSON 发到本机固定端口的调试绘图器
  （`src/utils/include/utils/plotter.hpp:14`），
  唯一引用是节点里一个**未使用**的成员声明，因此实际没有调用点。
- matchit 是一个模式匹配库（`src/utils/include/utils/matchit.h:16`），
  全仓库**零引用**。

**做法**：两者都属未使用代码，是否保留见 [待确认清单](./待确认清单.md)。

## Q8：为什么 `src/utils/src/utils.cpp` 几乎是空的？

**原因**：本模块是纯头文件库，所有实现都在头文件里
（`src/utils/src/utils.cpp:1`）。
该文件是 0 字节空文件：库为纯头文件实现，实现全部在头文件里。
校验器已允许对「存在但 0 行」的文件使用 `:1` 锚点，因此文档仍能锚定它。

## Q9：怎么新增一个 Eigen 类型别名？

**做法**：把别名加在别名头里，并基于统一的标量类型定义
（`src/utils/include/utils/eigen_alias.hpp:22`），
参照已有的定长/半动态/容器别名写法
（`src/utils/include/utils/eigen_alias.hpp:29-49`）。

**注意**：别名会被所有模块使用，属于公共契约，改动影响面最大；
若只是某个模块内部需要，建议先在该模块内定义（见 [待确认清单](./待确认清单.md)）。

## 排查手段

| 手段 | 用法 | 锚点 |
|---|---|---|
| 装载日志 | 绿/红/黄三色文本，含参数名与取值 | `src/utils/include/utils/yaml_loader.hpp:66` |
| 计时日志 | 过滤计时通道的 `[TIMER]` 前缀 | `src/utils/include/utils/scope_timer.hpp:73` |
| 按模块过滤 | 使用 7 个彩色通道之一 | `src/utils/include/utils/logger.hpp:119-125` |
| 静态检索 | 查宏重复定义与未使用头 | `src/utils/include/utils/type_utils.hpp:12` |
