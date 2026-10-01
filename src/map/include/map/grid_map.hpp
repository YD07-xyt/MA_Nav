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
    auto init(double x_left, double x_right, double y_left, double y_right, double resolution) -> void;
    //给定总尺寸，地图以原点为中心（等价于左=右=size/2）
    void init(double map_size_x, double map_size_y, double resolution) {
        init(map_size_x / 2.0, map_size_x / 2.0, map_size_y / 2.0, map_size_y / 2.0, resolution);
    }
    //TODO: 考虑地图中已有的障碍物点，不2次加入
    auto direct_set_map(const RowMatrixXi& map) -> void;
    void set_semantics();
    auto inflate(double radius) -> void;

    auto setMap(const RowMatrixXi& map) -> void;
    auto getDistance(const Eigen::Vector2d& pos) const -> double;
    auto getDistanceAndGradient(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d& gradient) const -> bool;
    auto isCollision(const Eigen::Vector2d& pos, double safe_threshold = 0.0) const -> bool;
    // bool isCollision(const Eigen::Vector2d& pos, double safe_threshold = 0.0) const {
    //     return !isInMap(pos) || getDistance(pos) < safe_threshold;
    // }

    auto isLineOccupancy(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const -> bool;

    auto getMap() const -> RowMatrixXd;

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

public:
    enum Semantics : uint8_t {
        NONE = 0,
        TUNNEL = 1,
    };
    // 设置某个栅格的标记
    auto set_semantics(const Eigen::Vector2i& idx, Semantics semantics) -> void {
        if (!isInMap(idx)) return;
        semantics_buffer_[toAddress(idx)] = static_cast<uint8_t>(semantics);
    }

    // 获取某个栅格的标记
    auto get_semantics(const Eigen::Vector2i& idx) const -> Semantics {
        if (!isInMap(idx)) return Semantics::NONE;
        return static_cast<Semantics>(semantics_buffer_[toAddress(idx)]);
    }

    //获取标记矩阵（用于可视化）
    auto get_semantics_map() const -> RowMatrixXi {
        RowMatrixXi semantics_map(voxel_num_.x(), voxel_num_.y());
        for (int x = 0; x < voxel_num_.x(); ++x)
            for (int y = 0; y < voxel_num_.y(); ++y)
                semantics_map(x, y) = semantics_buffer_[toAddress(x, y)];
        return semantics_map;
    }
    auto semantics_polygon_region(const std::vector<Eigen::Vector2d>& polygon, Semantics semantics) -> void {
        // 遍历所有栅格，将多边形内的栅格标记设为 marker
        for (int x = 0; x < voxel_num_.x(); ++x) {
            for (int y = 0; y < voxel_num_.y(); ++y) {
                Eigen::Vector2d pos;
                indexToPos(Eigen::Vector2i(x, y), pos);
                if (point_in_polygon(pos, polygon)) {
                    set_semantics({x, y}, semantics);
                }
            }
        }
    }

    // 可选：清除所有标记
    auto clear_all_markers() -> void {
        std::fill(semantics_buffer_.begin(), semantics_buffer_.end(), static_cast<uint8_t>(Semantics::NONE));
    }

    auto is_tunnel(const Eigen::Vector2d& pos) const -> bool {
        if (!isInMap(pos)) return false;
        Eigen::Vector2i idx;
        posToIndex(pos, idx);
        return get_semantics(idx) == Semantics::TUNNEL;
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
    auto inflateOccupancy(const RowMatrixXi& occ, double inflate_radius, double resolution) -> RowMatrixXi;
    auto updateESDF() -> void;

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

private:
    // 判断点是否在多边形内
    auto point_in_polygon(const Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& polygon) -> bool {
        bool inside = false;
        int n = polygon.size();
        for (int i = 0, j = n - 1; i < n; j = i++) {
            const Eigen::Vector2d& pi = polygon[i];
            const Eigen::Vector2d& pj = polygon[j];
            bool intersect = ((pi.y() > p.y()) != (pj.y() > p.y()))
                && (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) / (pj.y() - pi.y()) + pi.x());
            if (intersect) inside = !inside;
        }
        return inside;
    }

private:
    double resolution_, resolution_inv_;
    Eigen::Vector2d map_size_, map_origin_;
    Eigen::Vector2d min_boundary_, max_boundary_;
    Eigen::Vector2i voxel_num_;
    int buffer_size_;
    std::vector<char> occ_buffer_;
    std::vector<double> esdf_buffer_;
    std::vector<uint8_t> semantics_buffer_; // 每个元素存储 Semantics 的整数值
};

} // namespace grid_map