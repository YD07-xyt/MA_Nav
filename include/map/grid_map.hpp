/*
    MIT License

    Copyright (c) 2025 Senming Tan (senmingtan5@gmail.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#pragma once

#include <vector>
#include <Eigen/Eigen>
#include <memory>
#include <limits>
#include <iostream>

namespace grid_map {

using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using RowMatrixXi = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

class GridMap {
public:
    typedef std::shared_ptr<GridMap> Ptr;

    double getResolution() const {
        return resolution_;
    }
    Eigen::Vector2d getMapSize() const {
        return map_size_;
    }
    Eigen::Vector2d getOrigin() const {
        return map_origin_;
    }

    bool isInsideMap(const Eigen::Vector2d& pos) const {
        return (pos.array() >= min_boundary_.array()).all() && (pos.array() <= max_boundary_.array()).all();
    }

    // 按原点向四个方向的延伸量初始化地图边界：
    //   x ∈ [-x_left, +x_right]，y ∈ [-y_left, +y_right]
    void init(double x_left, double x_right, double y_left, double y_right, double resolution) {
        resolution_ = resolution;
        resolution_inv_ = 1.0 / resolution;
        min_boundary_ << -x_left, -y_left;
        max_boundary_ << x_right, y_right;
        map_origin_ = min_boundary_;
        map_size_ = max_boundary_ - min_boundary_;

        voxel_num_ = (map_size_ / resolution_).array().ceil().cast<int>();
        buffer_size_ = voxel_num_.x() * voxel_num_.y();

        occ_buffer_.resize(buffer_size_, 0);
        esdf_buffer_.resize(buffer_size_, 0.0);
    }
    // 兼容旧接口：给定总尺寸，地图以原点为中心（等价于左=右=size/2）
    void init(double map_size_x, double map_size_y, double resolution) {
        init(map_size_x / 2.0, map_size_x / 2.0, map_size_y / 2.0, map_size_y / 2.0, resolution);
    }
    //TODO: 考虑地图中已有的障碍物点，不2次加入
    void direct_set_map(const RowMatrixXi& map) {
        if (!checkMapSize(map)) return;
        for (int x = 0; x < voxel_num_.x(); ++x) {
            for (int y = 0; y < voxel_num_.y(); ++y) {
                occ_buffer_[toAddress(Eigen::Vector2i(x, y))] = map(x, y);
            }
        }
        updateESDF();
    }
    void inflate(double radius) {
        const auto voxel_num = getVoxelNum();
        const double res = getResolution();

        RowMatrixXi occ(voxel_num.x(), voxel_num.y());
        for (int x = 0; x < voxel_num.x(); ++x) {
            for (int y = 0; y < voxel_num.y(); ++y) {
                occ(x, y) = isOccupied({x, y}) ? 1 : 0;
            }
        }

        auto inflated = inflateOccupancy(occ, radius, res);

        setMap(inflated);
    }

    void setMap(const RowMatrixXi& map) {
        if (!checkMapSize(map)) return;
        bool changed = false;
        for (int x = 0; x < voxel_num_.x(); ++x) {
            for (int y = 0; y < voxel_num_.y(); ++y) {
                int addr = toAddress(Eigen::Vector2i(x, y));
                char new_val = static_cast<char>(map(x, y));
                if (occ_buffer_[addr] != new_val) {
                    occ_buffer_[addr] = new_val;
                    changed = true;
                }
            }
        }
        if (changed) {
            updateESDF();
        }
    }

    double getDistance(const Eigen::Vector2d& pos) const {
        if (!isInMap(pos)) return std::numeric_limits<double>::max();

        Eigen::Vector2d pos_m = pos;
        pos_m(0) -= 0.5 * resolution_;
        pos_m(1) -= 0.5 * resolution_;

        Eigen::Vector2i idx;
        posToIndex(pos_m, idx);

        Eigen::Vector2d idx_pos;
        indexToPos(idx, idx_pos);

        Eigen::Vector2d diff = pos - idx_pos;
        diff(0) *= resolution_inv_;
        diff(1) *= resolution_inv_;

        double values[2][2];
        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
                Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
                boundIndex(current_idx);
                values[x][y] = esdf_buffer_[toAddress(current_idx)];
            }
        }

        double v0 = values[0][0] * (1 - diff[0]) + values[1][0] * diff[0];
        double v1 = values[0][1] * (1 - diff[0]) + values[1][1] * diff[0];
        return v0 * (1 - diff[1]) + v1 * diff[1];
    }

    bool getDistanceAndGradient(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d& gradient) const {
        if (!isInMap(pos)) {
            distance = std::numeric_limits<double>::max();
            gradient.setZero();
            return false;
        }

        Eigen::Vector2d pos_m = pos;
        pos_m(0) -= 0.5 * resolution_;
        pos_m(1) -= 0.5 * resolution_;

        Eigen::Vector2i idx;
        posToIndex(pos_m, idx);

        Eigen::Vector2d idx_pos;
        indexToPos(idx, idx_pos);

        Eigen::Vector2d diff = pos - idx_pos;
        diff(0) *= resolution_inv_;
        diff(1) *= resolution_inv_;

        double values[2][2];
        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
                Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
                boundIndex(current_idx);
                values[x][y] = esdf_buffer_[toAddress(current_idx)];
            }
        }

        double v0 = values[0][0] * (1 - diff[0]) + values[1][0] * diff[0];
        double v1 = values[0][1] * (1 - diff[0]) + values[1][1] * diff[0];
        distance = v0 * (1 - diff[1]) + v1 * diff[1];

        gradient(1) = (v1 - v0) * resolution_inv_;
        gradient(0) = (1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1]);
        gradient(0) *= resolution_inv_;

        return true;
    }

    // bool isCollision(const Eigen::Vector2d& pos, double safe_threshold = 0.0) const {
    //     return !isInMap(pos) || getDistance(pos) < safe_threshold;
    // }
    bool isCollision(const Eigen::Vector2d& pos, double safe_threshold = 0.0) const {
        if (!isInMap(pos)) {
            return false;
        }
        return getDistance(pos) < safe_threshold;
    }
    bool isLineOccupancy(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const {
        Eigen::Vector2d diff = p2 - p1;
        double max_dist = diff.norm();
        if (max_dist < 1e-6) return isCollision(p1);

        Eigen::Vector2d dir = diff.normalized();
        double step = resolution_ * 0.1;

        for (double d = 0; d <= max_dist; d += step) {
            Eigen::Vector2d pt = p1 + dir * d;
            if (isCollision(pt)) return true;
        }
        return false;
    }

    RowMatrixXd getMap() const {
        RowMatrixXd map(voxel_num_.x(), voxel_num_.y());
        for (int x = 0; x < voxel_num_.x(); ++x) {
            for (int y = 0; y < voxel_num_.y(); ++y) {
                map(x, y) = esdf_buffer_[toAddress(Eigen::Vector2i(x, y))];
            }
        }
        return map;
    }

    Eigen::Vector2i getVoxelNum() const {
        return voxel_num_;
    }

    bool isOccupied(const Eigen::Vector2i& id) const {
        return isInMap(id) ? occ_buffer_[toAddress(id)] == 1 : true;
    }

    void posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& id) const {
        id << static_cast<int>((pos.x() - map_origin_.x()) * resolution_inv_),
            static_cast<int>((pos.y() - map_origin_.y()) * resolution_inv_);
    }

    void indexToPos(const Eigen::Vector2i& id, Eigen::Vector2d& pos) const {
        pos << (id.x() + 0.5) * resolution_ + map_origin_.x(), (id.y() + 0.5) * resolution_ + map_origin_.y();
    }

protected:
    inline int toAddress(int x, int y) const {
        return x * voxel_num_.y() + y;
    }

    // 保护 setMap/direct_set_map 不越界：输入矩阵行列必须与当前栅格一致，
    // 否则直接拒绝（尺寸不一致时按像素索引拷贝会导致错位或读越界内存）
    bool checkMapSize(const RowMatrixXi& map) const {
        if (map.rows() != voxel_num_.x() || map.cols() != voxel_num_.y()) {
            std::cerr << "[GridMap] size mismatch: got " << map.rows() << "x" << map.cols() << ", expected "
                      << voxel_num_.x() << "x" << voxel_num_.y() << ", map update rejected" << std::endl;
            return false;
        }
        return true;
    }
    grid_map::RowMatrixXi inflateOccupancy(const grid_map::RowMatrixXi& occ, double inflate_radius, double resolution) {
        const int rows = occ.rows();
        const int cols = occ.cols();

        // 膨胀半径对应的格子数
        int r = static_cast<int>(std::ceil(inflate_radius / resolution));

        // 拷贝一份，避免影响原始 occupancy
        grid_map::RowMatrixXi inflated = occ;

        for (int x = 0; x < rows; ++x) {
            for (int y = 0; y < cols; ++y) {
                if (occ(x, y) != 1) continue;

                // 圆形膨胀
                for (int dx = -r; dx <= r; ++dx) {
                    for (int dy = -r; dy <= r; ++dy) {
                        if (dx * dx + dy * dy > r * r) continue;

                        int nx = x + dx;
                        int ny = y + dy;

                        if (nx < 0 || nx >= rows || ny < 0 || ny >= cols) {
                            continue;
                        }

                        inflated(nx, ny) = 1;
                    }
                }
            }
        }

        return inflated;
    }
    void updateESDF() {
        int rows = voxel_num_.x();
        int cols = voxel_num_.y();

        RowMatrixXd tmp_buffer(rows, cols);
        RowMatrixXd neg_buffer(rows, cols);
        RowMatrixXi neg_map(rows, cols);
        RowMatrixXd dist_buffer(rows, cols);

        for (int x = 0; x < rows; ++x) {
            fillESDF(
                [&](int y) {
                    return occ_buffer_[toAddress(Eigen::Vector2i(x, y))] ? 0 : std::numeric_limits<double>::max();
                },
                [&](int y, double val) { tmp_buffer(x, y) = val; },
                0,
                cols - 1,
                cols
            );
        }

        for (int y = 0; y < cols; ++y) {
            fillESDF(
                [&](int x) { return tmp_buffer(x, y); },
                [&](int x, double val) { dist_buffer(x, y) = resolution_ * std::sqrt(val); },
                0,
                rows - 1,
                rows
            );
        }

        for (int x = 0; x < rows; ++x) {
            for (int y = 0; y < cols; ++y) {
                neg_map(x, y) = (occ_buffer_[toAddress(Eigen::Vector2i(x, y))] == 0) ? 1 : 0;
            }
        }

        for (int x = 0; x < rows; ++x) {
            fillESDF(
                [&](int y) { return neg_map(x, y) ? 0 : std::numeric_limits<double>::max(); },
                [&](int y, double val) { tmp_buffer(x, y) = val; },
                0,
                cols - 1,
                cols
            );
        }

        for (int y = 0; y < cols; ++y) {
            fillESDF(
                [&](int x) { return tmp_buffer(x, y); },
                [&](int x, double val) { neg_buffer(x, y) = resolution_ * std::sqrt(val); },
                0,
                rows - 1,
                rows
            );
        }

        for (int x = 0; x < voxel_num_.x(); ++x) {
            for (int y = 0; y < voxel_num_.y(); ++y) {
                double pos_dist = dist_buffer(x, y);
                double neg_dist = std::abs(neg_buffer(x, y));

                if (neg_dist > 0.0) {
                    esdf_buffer_[toAddress(x, y)] = -neg_dist + resolution_;
                } else {
                    esdf_buffer_[toAddress(x, y)] = pos_dist;
                }
            }
        }
    }

private:
    template<typename F_get_val, typename F_set_val>
    static void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int size) {
        std::vector<int> v(size);
        std::vector<double> z(size + 1);

        int k = start;
        v[start] = start;
        z[start] = -std::numeric_limits<double>::max();
        z[start + 1] = std::numeric_limits<double>::max();

        for (int q = start + 1; q <= end; q++) {
            k++;
            double s;

            do {
                k--;
                s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
            } while (s <= z[k]);

            k++;
            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<double>::max();
        }

        k = start;
        for (int q = start; q <= end; q++) {
            while (z[k + 1] < q)
                k++;
            double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
            f_set_val(q, val);
        }
    }

    inline int toAddress(const Eigen::Vector2i& id) const {
        return id.x() * voxel_num_.y() + id.y();
    }

    inline void boundIndex(Eigen::Vector2i& id) const {
        id(0) = std::max(std::min(id(0), voxel_num_.x() - 1), 0);
        id(1) = std::max(std::min(id(1), voxel_num_.y() - 1), 0);
    }

    inline bool isInMap(const Eigen::Vector2d& pos) const {
        return (pos.array() >= min_boundary_.array()).all() && (pos.array() <= max_boundary_.array()).all();
    }

    inline bool isInMap(const Eigen::Vector2i& idx) const {
        return idx.x() >= 0 && idx.y() >= 0 && idx.x() < voxel_num_.x() && idx.y() < voxel_num_.y();
    }

    double resolution_, resolution_inv_;
    Eigen::Vector2d map_size_, map_origin_;
    Eigen::Vector2d min_boundary_, max_boundary_;
    Eigen::Vector2i voxel_num_;
    int buffer_size_;
    std::vector<char> occ_buffer_;
    std::vector<double> esdf_buffer_;
};

} // namespace grid_map