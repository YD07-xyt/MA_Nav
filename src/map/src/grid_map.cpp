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
    setMap(map);
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

auto GridMap::sampleEsdf(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d* gradient) const -> bool {
    if (!isInMap(pos)) return false;

    const Eigen::Vector2d local = (pos - map_origin_) * resolution_inv_;
    const Eigen::Vector2i center(static_cast<int>(std::lround(local.x() - 0.5)), static_cast<int>(std::lround(local.y() - 0.5)));
    // 二次拟合以最近格心为 0。双线性仍用该格心西南角，分数坐标落在 [0,1]。
    const double tx = local.x() - 0.5 - center.x();
    const double ty = local.y() - 0.5 - center.y();
    // 双线性的 2×2 角点仍是原实现的采样格，不随最近格心移动。
    Eigen::Vector2d pos_m = pos - 0.5 * resolution_ * Eigen::Vector2d::Ones();
    Eigen::Vector2i idx;
    posToIndex(pos_m, idx);
    Eigen::Vector2d idx_pos;
    indexToPos(idx, idx_pos);
    const double fx = (pos.x() - idx_pos.x()) * resolution_inv_;
    const double fy = (pos.y() - idx_pos.y()) * resolution_inv_;

    auto sampleBilinear = [&]() {
        double values[2][2];
        for (int x = 0; x < 2; ++x) {
            for (int y = 0; y < 2; ++y) {
                Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
                boundIndex(current_idx);
                values[x][y] = esdf_buffer_[toAddress(current_idx)];
            }
        }
        const double v0 = values[0][0] * (1.0 - fx) + values[1][0] * fx;
        const double v1 = values[0][1] * (1.0 - fx) + values[1][1] * fx;
        distance = v0 * (1.0 - fy) + v1 * fy;
        if (gradient) {
            (*gradient)(1) = (v1 - v0) * resolution_inv_;
            (*gradient)(0) =
                ((1.0 - fy) * (values[1][0] - values[0][0]) + fy * (values[1][1] - values[0][1])) * resolution_inv_;
        }
    };

    // 3x3 需要中心格四周各一格；贴边时邻域不完整，退回双线性。
    if (center.x() < 1 || center.y() < 1 || center.x() + 1 >= voxel_num_.x() || center.y() + 1 >= voxel_num_.y()) {
        sampleBilinear();
        return true;
    }

    const double wx[3] = {0.5 * tx * (tx - 1.0), 1.0 - tx * tx, 0.5 * tx * (tx + 1.0)};
    const double wy[3] = {0.5 * ty * (ty - 1.0), 1.0 - ty * ty, 0.5 * ty * (ty + 1.0)};
    const double dwx[3] = {tx - 0.5, -2.0 * tx, tx + 0.5};
    const double dwy[3] = {ty - 0.5, -2.0 * ty, ty + 0.5};

    distance = 0.0;
    double ddx = 0.0;
    double ddy = 0.0;
    for (int dy = 0; dy < 3; ++dy) {
        for (int dx = 0; dx < 3; ++dx) {
            const double d = esdf_buffer_[toAddress(center + Eigen::Vector2i(dx - 1, dy - 1))];
            distance += wx[dx] * wy[dy] * d;
            if (gradient) {
                ddx += dwx[dx] * wy[dy] * d;
                ddy += wx[dx] * dwy[dy] * d;
            }
        }
    }
    if (gradient) {
        gradient->x() = ddx * resolution_inv_;
        gradient->y() = ddy * resolution_inv_;
    }
    return true;
}

auto GridMap::getDistance(const Eigen::Vector2d& pos) const -> double {
    double distance = 0.0;
    if (!sampleEsdf(pos, distance, nullptr)) return std::numeric_limits<double>::max();
    return distance;
}

auto GridMap::getDistanceAndGradient(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d& gradient) const
    -> bool {
    if (!sampleEsdf(pos, distance, &gradient)) {
        distance = std::numeric_limits<double>::max();
        gradient.setZero();
        return false;
    }
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