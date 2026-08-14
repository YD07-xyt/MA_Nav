#include "map/terrain_analysis.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace Terrain {

auto TerrainAnalyzer::analyze(const utils::RobotState &robot_pose,
                              const Map &occ_map) -> Map {
  // 1. 确定分析窗口
  GridInfo grid = setupGrid(robot_pose.p);

  // 2. 将点云分配到网格，记录每个网格的最低点和最高点
  std::vector<std::vector<float>> height_lists(grid.width * grid.height);
  for (const auto &pt : occ_map) {
    auto idx_opt = worldToGrid(grid, pt);
    if (!idx_opt) continue;
    auto [gx, gy] = *idx_opt;
    height_lists[gy * grid.width + gx].push_back(pt.z());
  }

  // 3. 提取网格内的 最低点（作为地面） 和 最高点（作为墙体）
  Eigen::ArrayXXf ground = Eigen::ArrayXXf::Constant(
      grid.width, grid.height, std::numeric_limits<float>::quiet_NaN());
  Eigen::ArrayXXf max_height = Eigen::ArrayXXf::Constant(
      grid.width, grid.height, std::numeric_limits<float>::quiet_NaN());

  for (int gy = 0; gy < grid.height; ++gy) {
    for (int gx = 0; gx < grid.width; ++gx) {
      const auto &z_vals = height_lists[gy * grid.width + gx];
      if (z_vals.empty()) continue;

      float min_z = std::numeric_limits<float>::max();
      float max_z = -std::numeric_limits<float>::max();
      for (float z : z_vals) {
        if (z < min_z) min_z = z;
        if (z > max_z) max_z = z;
      }
      ground(gx, gy) = min_z;
      max_height(gx, gy) = max_z;
    }
  }

  // 4. 中值滤波平滑地面高度
  ground = medianFilter(ground, config_.kernel_size);

  // =================计算邻域高度梯度 =================

  Eigen::ArrayXXf gradient = Eigen::ArrayXXf::Zero(grid.width, grid.height);

  for (int gy = 0; gy < grid.height; ++gy) {
    for (int gx = 0; gx < grid.width; ++gx) {
      if (std::isnan(ground(gx, gy))) continue;
      float max_diff = 0.0f;
      // 遍历8邻域
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) continue;
          int nx = gx + dx;
          int ny = gy + dy;
          if (nx < 0 || nx >= grid.width || ny < 0 || ny >= grid.height)
          continue; float neighbor = ground(nx, ny); if
          (std::isnan(neighbor)) continue; float diff = std::abs(ground(gx,
          gy) - neighbor); if (diff > max_diff) max_diff = diff;
        }
      }
      gradient(gx, gy) = max_diff;
    }
  }
  // =========================================================

  // 综合判断并输出障碍物
  Map output;
  output.reserve(grid.width * grid.height);

  for (int gy = 0; gy < grid.height; ++gy) {
    for (int gx = 0; gx < grid.width; ++gx) {
      float current_ground = ground(gx, gy);
      if (std::isnan(current_ground)) continue;

      float obstacle_max_z = max_height(gx, gy);
      if (std::isnan(obstacle_max_z)) continue;

      float high_diff = obstacle_max_z - current_ground;
      bool is_steep_edge = (gradient(gx, gy) > config_.steep_threshold);

      // 【情况A】邻域存在陡峭高度突变 → 直接判定为垂直障碍（墙/悬崖）
      if (is_steep_edge) {
        auto world_pt = gridToWorld(grid, gx, gy);
        world_pt.z() = obstacle_max_z; // 保留墙体/悬崖顶部的高度
        output.push_back(world_pt);
        continue; // 直接跳过后续间隙检查
      }

      //【情况B】非陡峭边缘，仅凭内部高度差判断（保留原有逻辑，用于识别孤立高柱/台阶）
      if (high_diff <= config_.max_step_height)
        continue;

      // 间隙分析（原逻辑）
      const auto &z_vals = height_lists[gy * grid.width + gx];
      if (z_vals.empty()) continue;

      std::vector<float> heights;
      heights.reserve(z_vals.size());
      for (float z : z_vals) {
        if (z >= current_ground - 0.05f) { // 过滤过低噪声
          heights.push_back(z);
        }
      }
      if (heights.empty()) continue;

      std::sort(heights.begin(), heights.end());
      float last_z = current_ground;
      float max_gap = 0.0f;
      for (float z : heights) {
        if (z <= last_z) continue;
        float gap = z - last_z;
        if (gap > max_gap) max_gap = gap;
        last_z = z;
      }

      bool blocked = (max_gap < config_.robot_height);
      if (blocked && obstacle_max_z > 0.15f) {
        auto world_pt = gridToWorld(grid, gx, gy);
        world_pt.z() = obstacle_max_z;
        output.push_back(world_pt);
      }
    }
  }
  return output;
}
// auto TerrainAnalyzer::analyze(const utils::RobotState &robot_pose,
//                               const Map &occ_map) -> Map {
//   GridInfo grid = setupGrid(robot_pose.p);

//   // 1. 存储每个网格内的完整点（Vec3f）
//   std::vector<std::vector<utils::Vec3f>> points_in_grid(grid.width *
//                                                           grid.height);
//   for (const auto &pt : occ_map) {
//     auto idx_opt = worldToGrid(grid, pt);
//     if (!idx_opt)
//       continue;
//     auto [gx, gy] = *idx_opt;
//     points_in_grid[gy * grid.width + gx].push_back(pt);
//   }

//   // 2. 提取最低点（地面）和最高点
//   Eigen::ArrayXXf ground = Eigen::ArrayXXf::Constant(
//       grid.width, grid.height, std::numeric_limits<float>::quiet_NaN());
//   Eigen::ArrayXXf max_height = Eigen::ArrayXXf::Constant(
//       grid.width, grid.height, std::numeric_limits<float>::quiet_NaN());

//   for (int gy = 0; gy < grid.height; ++gy) {
//     for (int gx = 0; gx < grid.width; ++gx) {
//       const auto &pts = points_in_grid[gy * grid.width + gx];
//       if (pts.empty())
//         continue;

//       float min_z = std::numeric_limits<float>::max();
//       float max_z = -std::numeric_limits<float>::max();
//       for (const auto &pt : pts) {
//         float z = pt.z();
//         if (z < min_z)
//           min_z = z;
//         if (z > max_z)
//           max_z = z;
//       }
//       ground(gx, gy) = min_z;
//       max_height(gx, gy) = max_z;
//     }
//   }

//   // 3. 中值滤波
//   ground = medianFilter(ground, config_.kernel_size);

//   // 4. 计算梯度（保持不变）
//   Eigen::ArrayXXf gradient = Eigen::ArrayXXf::Zero(grid.width, grid.height);
//   for (int gy = 0; gy < grid.height; ++gy) {
//     for (int gx = 0; gx < grid.width; ++gx) {
//       if (std::isnan(ground(gx, gy)))
//         continue;
//       float max_diff = 0.0f;
//       for (int dy = -1; dy <= 1; ++dy) {
//         for (int dx = -1; dx <= 1; ++dx) {
//           if (dx == 0 && dy == 0)
//             continue;
//           int nx = gx + dx;
//           int ny = gy + dy;
//           if (nx < 0 || nx >= grid.width || ny < 0 || ny >= grid.height)
//             continue;
//           float neighbor = ground(nx, ny);
//           if (std::isnan(neighbor))
//             continue;
//           float diff = std::abs(ground(gx, gy) - neighbor);
//           if (diff > max_diff)
//             max_diff = diff;
//         }
//       }
//       gradient(gx, gy) = max_diff;
//     }
//   }

//   // 5. 综合判断并输出障碍物（输出原始点）
//   Map output;
//   output.reserve(grid.width * grid.height * 10); // 预留空间

//   for (int gy = 0; gy < grid.height; ++gy) {
//     for (int gx = 0; gx < grid.width; ++gx) {
//       float current_ground = ground(gx, gy);
//       if (std::isnan(current_ground))
//         continue;

//       float obstacle_max_z = max_height(gx, gy);
//       if (std::isnan(obstacle_max_z))
//         continue;

//       float high_diff = obstacle_max_z - current_ground;
//       bool is_steep_edge = (gradient(gx, gy) > config_.steep_threshold);

//       // 【情况A】陡峭边缘 → 输出所有高于地面的点
//       if (is_steep_edge) {
//         for (const auto &pt : points_in_grid[gy * grid.width + gx]) {
//           if (pt.z() > current_ground + 0.05f) {
//             output.push_back(pt);
//           }
//         }
//         continue;
//       }

//       // 【情况B】非陡峭，但内部高差过大 → 进一步做间隙分析
//       if (high_diff <= config_.max_step_height)
//         continue;

//       // 间隙分析（原逻辑，需使用存储的点进行高度排序）
//       const auto &pts = points_in_grid[gy * grid.width + gx];
//       if (pts.empty())
//         continue;

//       std::vector<float> heights;
//       heights.reserve(pts.size());
//       for (const auto &pt : pts) {
//         float z = pt.z();
//         if (z >= current_ground - 0.05f) { // 过滤过低噪声
//           heights.push_back(z);
//         }
//       }
//       if (heights.empty())
//         continue;

//       std::sort(heights.begin(), heights.end());
//       float last_z = current_ground;
//       float max_gap = 0.0f;
//       for (float z : heights) {
//         if (z <= last_z)
//           continue;
//         float gap = z - last_z;
//         if (gap > max_gap)
//           max_gap = gap;
//         last_z = z;
//       }

//       bool blocked = (max_gap < config_.robot_height);
//       if (blocked && obstacle_max_z > 0.15f) {
//         // 输出该网格内所有高于地面的点
//         for (const auto &pt : pts) {
//           if (pt.z() > current_ground + 0.05f) {
//             output.push_back(pt);
//           }
//         }
//       }
//     }
//   }
//   return output;
// }
// ========== 工具函数实现 ==========

auto TerrainAnalyzer::setupGrid(const utils::Vec3f &robot_pos) const
    -> GridInfo {
  GridInfo grid;
  // 将分析窗口原点对齐到 resolution 网格上，保证网格中心在世界坐标系下
  // 保持固定，避免机器人连续运动时地形点发生亚网格级抖动。
  const float res = config_.resolution;
  const float ox =
      std::round((robot_pos.x() - config_.map_size_x * 0.5f) / res) * res;
  const float oy =
      std::round((robot_pos.y() - config_.map_size_y * 0.5f) / res) * res;
  grid.origin = utils::Vec3f(ox, oy, 0.0f);
  grid.width =
      static_cast<int>(std::ceil(config_.map_size_x / config_.resolution));
  grid.height =
      static_cast<int>(std::ceil(config_.map_size_y / config_.resolution));
  return grid;
}

auto TerrainAnalyzer::worldToGrid(const GridInfo &grid,
                                  const utils::Vec3f &world) const
    -> std::optional<std::pair<int, int>> {
  int gx = static_cast<int>(
      std::floor((world.x() - grid.origin.x()) / config_.resolution));
  int gy = static_cast<int>(
      std::floor((world.y() - grid.origin.y()) / config_.resolution));
  if (gx >= 0 && gx < grid.width && gy >= 0 && gy < grid.height) {
    return std::make_pair(gx, gy);
  }
  return std::nullopt;
}

auto TerrainAnalyzer::gridToWorld(const GridInfo &grid, int gx, int gy) const
    -> utils::Vec3f {
  float wx =
      grid.origin.x() + (static_cast<float>(gx) + 0.5f) * config_.resolution;
  float wy =
      grid.origin.y() + (static_cast<float>(gy) + 0.5f) * config_.resolution;
  return utils::Vec3f(wx, wy, 0.0f);
}

auto TerrainAnalyzer::medianFilter(const Eigen::ArrayXXf &input,
                                   int kernel_size) -> Eigen::ArrayXXf {
  int rows = input.rows();
  int cols = input.cols();
  Eigen::ArrayXXf output = Eigen::ArrayXXf::Constant(
      rows, cols, std::numeric_limits<float>::quiet_NaN());
  int k = kernel_size / 2;

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      std::vector<float> neighbors;
      neighbors.reserve(kernel_size * kernel_size);

      for (int m = -k; m <= k; ++m) {
        for (int n = -k; n <= k; ++n) {
          int ni = i + m;
          int nj = j + n;
          if (ni >= 0 && ni < rows && nj >= 0 && nj < cols) {
            float val = input(ni, nj);
            if (!std::isnan(val)) {
              neighbors.push_back(val);
            }
          }
        }
      }

      if (!neighbors.empty()) {
        std::sort(neighbors.begin(), neighbors.end());
        size_t mid = neighbors.size() / 2;
        if (neighbors.size() % 2 == 0) {
          output(i, j) = (neighbors[mid - 1] + neighbors[mid]) * 0.5f;
        } else {
          output(i, j) = neighbors[mid];
        }
      }
    }
  }
  return output;
}

} // namespace Terrain