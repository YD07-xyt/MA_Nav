/*
    MIT License

    Copyright (c) 2025 Senming Tan (senmingtan5@gmail.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/

#pragma once

#include "utils/scope_timer.hpp"
#include "map/grid_map.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <spdlog/spdlog.h>
#include <unordered_map>
#include <vector>

namespace path_planning {

class AStar {
public:
    // Node structure containing position and search state
    struct Node {
        Eigen::Vector2d position;
        double g_score = 0; // Actual cost from start to current node
        double f_score = 0; // g_score + heuristic estimate
        Node* parent = nullptr;

        bool operator>(const Node& other) const {
            return f_score > other.f_score;
        }
    };

    AStar(grid_map::GridMap& map, double safe_threshold);
    AStar()=default;

    auto set_map(const grid_map::GridMap& map)-> void;
    auto set_safe_threshold(const double& safe_threshold)->void{
      safe_threshold_=safe_threshold;
    }
    auto original_astar_search(const Eigen::Vector2d& start, const Eigen::Vector2d& goal, int timeout_ms = 1000)
        -> std::vector<Eigen::Vector2d>;


private:
    grid_map::GridMap map_;
    double safe_threshold_;
    // Euclidean heuristic
    auto heuristic(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const -> double;

    // Position to hash key
    auto position_to_key(const Eigen::Vector2d& pos) const -> std::string;

    // Reconstruct path
    auto reconstruct_path(Node* node) const -> std::vector<Eigen::Vector2d>;

    auto check_collision(const Eigen::Vector2d& pos) const -> bool;

};

} // namespace path_planning