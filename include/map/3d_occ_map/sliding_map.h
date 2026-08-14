/**
* This file is part of ROG-Map
*
* Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/ROG-Map>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* ROG-Map is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ROG-Map is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

// #include <rog_map/rog_map_core/config.hpp>
#include "../../utils/scope_timer.hpp"
#include "../../utils/eigen_alias.hpp"
#include <Eigen/Core>

namespace rog_map {
    /// The policy of ORIGIN_AT_CORNER is:
    //    for all cells, the pos is defined at the center of the cell
    //    including the map boundaries.
    //    [ORIGIN] And for the origin, it is defined on the cornerQ of the cell
    //      bd_min       origin           bd_max
    //        |              |              |
    //        v              V              V
    //        -2        -1        0         1
    //   |---------|---------|---------|---------|
    //  -2  -1.5  -1    0.5  0   0.5   1   1.5   2

    class SlidingMap {
    public:
        //用于解决在堆上动态分配含有固定大小向量化 Eigen 对象的类时，因内存对齐不足导致的崩溃问题
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SlidingMap(const utils::Vec3i &half_map_size_i,
                   const double &resolution,
                   const bool &map_sliding_en,
                   const double &sliding_thresh,
                   const utils::Vec3f &fix_map_origin);

        SlidingMap() = default;

        virtual ~SlidingMap() = default;

        void initSlidingMap(const utils::Vec3i &half_map_size_i,
                  const double &resolution,
                  const bool &map_sliding_en,
                  const double &sliding_thresh,
                  const utils::Vec3f &fix_map_origin);

        void printMapInformation();

        void mapSliding(const utils::Vec3f &odom);

        bool insideLocalMap(const utils::Vec3f &pos) const;

        bool insideLocalMap(const utils::Vec3i &id_g) const;

    protected:
        struct SlidingConfig {
            double resolution{0};
            double resolution_inv{0};
            double sliding_thresh{0};
            bool map_sliding_en{false};
            utils::Vec3f fix_map_origin{};
            utils::Vec3i visualization_range_i{};
            utils::Vec3i map_size_i{};
            utils::Vec3i half_map_size_i{};
            int virtual_ceil_height_id_g{0};
            int virtual_ground_height_id_g{0};
            int safe_margin_i{0};
            int map_vox_num{0};
        } sc_;

        utils::Vec3f local_map_origin_d_, local_map_bound_min_d_, local_map_bound_max_d_;
        utils::Vec3i local_map_origin_i_, local_map_bound_min_i_, local_map_bound_max_i_;

        virtual void resetLocalMap() = 0;

        /* When map sliding, the memory out of the local map should be cleared,
         * which cause a cell state jumped to unknown. Thus, the method of resetCell
         * should be implemented in the derived class.
         *
         * In ROG Map, this api is only for ProbMap and CounterMap, for the derived classes
         * from counter map, check the resetOneCell method in CounterMap.
         * */

        virtual void resetCell(const int & hash_id) = 0;

        void clearMemoryOutOfMap(const std::vector<int> &clear_id, const int &i);


        int getLocalIndexHash(const utils::Vec3i &id_in) const;

        void posToGlobalIndex(const utils::Vec3f &pos, utils::Vec3i &id) const;

        void posToGlobalIndex(const double &pos, int &id) const;

        void globalIndexToPos(const utils::Vec3i &id_g, utils::Vec3f &pos) const;

        void globalIndexToLocalIndex(const utils::Vec3i &id_g, utils::Vec3i &id_l) const;

        /* Only used in clearMemoryOutOfMap and */
        void localIndexToGlobalIndex(const utils::Vec3i &id_l, utils::Vec3i &id_g) const;

        void localIndexToPos(const utils::Vec3i &id_l, utils::Vec3f &pos) const;

        void hashIdToLocalIndex(const int &hash_id,
                                utils::Vec3i &id) const;

        void hashIdToPos(const int &hash_id,
                         utils::Vec3f &pos) const;

        void hashIdToGlobalIndex(const int &hash_id,
                                 utils::Vec3i &id_g) const;

        int getHashIndexFromPos(const utils::Vec3f &pos) const;

        int getHashIndexFromGlobalIndex(const utils::Vec3i &id_g) const;

        void updateLocalMapOriginAndBound(const utils::Vec3f &new_origin_d,
                                          const utils::Vec3i &new_origin_i);


    private:
        bool had_been_initialized{false};

    };


}