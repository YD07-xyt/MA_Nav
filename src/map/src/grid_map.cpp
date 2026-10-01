#include "map/grid_map.hpp"
namespace grid_map {
auto GridMap::init(double x_left, double x_right, double y_left, double y_right, double resolution) -> void {
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
    semantics_buffer_.resize(buffer_size_, static_cast<uint8_t>(Semantics::NONE));
}
auto GridMap::direct_set_map(const RowMatrixXi& map) -> void {
    if (!checkMapSize(map)) return;
    for (int x = 0; x < voxel_num_.x(); ++x) {
        for (int y = 0; y < voxel_num_.y(); ++y) {
            occ_buffer_[toAddress(Eigen::Vector2i(x, y))] = map(x, y);
        }
    }
    updateESDF();
}
auto GridMap::inflate(double radius) -> void {
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
auto GridMap::setMap(const RowMatrixXi& map) -> void {
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

auto GridMap::getDistance(const Eigen::Vector2d& pos) const -> double {
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

auto GridMap::getDistanceAndGradient(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d& gradient) const
    -> bool {
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
auto GridMap::isCollision(const Eigen::Vector2d& pos, double safe_threshold) const -> bool {
    if (!isInMap(pos)) {
        return false;
    }
    return getDistance(pos) < safe_threshold;
}
auto GridMap::isLineOccupancy(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const -> bool {
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

auto GridMap::getMap() const -> RowMatrixXd {
    RowMatrixXd map(voxel_num_.x(), voxel_num_.y());
    for (int x = 0; x < voxel_num_.x(); ++x) {
        for (int y = 0; y < voxel_num_.y(); ++y) {
            map(x, y) = esdf_buffer_[toAddress(Eigen::Vector2i(x, y))];
        }
    }
    return map;
}
auto GridMap::inflateOccupancy(const RowMatrixXi& occ, double inflate_radius, double resolution) -> RowMatrixXi {
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
auto GridMap::updateESDF() -> void {
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
}