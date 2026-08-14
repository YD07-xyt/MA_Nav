#include "planner/path_planning/astar.h"
#include "utils/logger.hpp"

namespace path_planning {
AStar::AStar(grid_map::GridMap& map, double safe_threshold): map_(map), safe_threshold_(safe_threshold) {

};


auto AStar::set_map(const grid_map::GridMap& map) -> void {
    map_ = map;
}

auto AStar::original_astar_search(const Eigen::Vector2d& start, const Eigen::Vector2d& goal, int timeout_ms)
    -> std::vector<Eigen::Vector2d> {
    auto start_time = std::chrono::steady_clock::now();

    // Check start/goal validity
    if (check_collision(start)) {
        logger::planning->error("start point in collision!");
        return {};
    }
    if (check_collision(goal)) {
        logger::planning->error("Goal point in collision!");
        return {};
    }

    // Open set (priority queue)
    auto cmp = [](Node* a, Node* b) { return *a > *b; };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open_set(cmp);

    // Node storage and hash map
    std::vector<std::unique_ptr<Node>> nodes;
    std::unordered_map<std::string, Node*> node_map;

    // Start node
    nodes.emplace_back(new Node {start, 0, heuristic(start, goal)});
    open_set.push(nodes.back().get());
    node_map[position_to_key(start)] = nodes.back().get();

    while (!open_set.empty()) {
        // Check timeout
        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count()
            > timeout_ms)
        {
            logger::planning->error("A* timeout!");
            return {};
        }

        Node* current = open_set.top();
        open_set.pop();

        // Reached goal (using resolution as tolerance)
        if ((current->position - goal).norm() < map_.getResolution()) {
            return reconstruct_path(current);
        }

        // Expand neighbors (8 directions)
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;

                Eigen::Vector2d neighbor_pos = current->position + Eigen::Vector2d(dx, dy) * map_.getResolution();

                // Skip collision points
                if (check_collision(neighbor_pos)) {
                    continue;
                }

                // Calculate tentative_g_score
                double step_cost = (dx * dy == 0) ? 1.0 : 1.414; // Orthogonal/diagonal cost
                double tentative_g = current->g_score + step_cost * map_.getResolution();

                // Check if better path exists
                std::string key = position_to_key(neighbor_pos);
                auto it = node_map.find(key);
                if (it == node_map.end() || tentative_g < it->second->g_score) {
                    // New node or found better path
                    nodes.emplace_back(
                        new Node {neighbor_pos, tentative_g, tentative_g + heuristic(neighbor_pos, goal), current}
                    );
                    open_set.push(nodes.back().get());
                    node_map[key] = nodes.back().get();
                }
            }
        }
    }

    return {}; // Open set empty, no path found
}

auto AStar::heuristic(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const -> double {
    return (a - b).norm();
}

// Position to hash key
auto AStar::position_to_key(const Eigen::Vector2d& pos) const -> std::string {
    return std::to_string(static_cast<int>(pos.x() * 100)) + "," + std::to_string(static_cast<int>(pos.y() * 100));
}

// Reconstruct path
auto AStar::reconstruct_path(Node* node) const -> std::vector<Eigen::Vector2d> {
    std::vector<Eigen::Vector2d> path;
    while (node) {
        path.push_back(node->position);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

auto AStar::check_collision(const Eigen::Vector2d& pos) const -> bool {
    // Points outside map are considered collision-free
    if (!map_.isInsideMap(pos)) return false;

    // Points inside map use safety distance check
    return map_.getDistance(pos) < safe_threshold_;
}
}