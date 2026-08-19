#pragma once

#include <Eigen/Core>
/**
 * @brief ESDF 接口抽象类
 *
 * 用户需要实现此接口以提供环境距离场信息
 */
class ESDFInterface {
public:
    virtual ~ESDFInterface() = default;

    /**
     * @brief 获取点 (x, y) 到最近障碍物的有符号距离
     * @param x X 坐标（世界坐标系）
     * @param y Y 坐标（世界坐标系）
     * @return 有符号距离，正值表示在自由空间内，负值表示在障碍物内
     */
    virtual double getDistance(double x, double y) const = 0;

    /**
     * @brief 获取距离场在点 (x, y) 处的梯度
     * @param x X 坐标
     * @param y Y 坐标
     * @return 梯度向量 (∂d/∂x, ∂d/∂y)
     */
    virtual Eigen::Vector2d getGradient(double x, double y) const = 0;

    /**
     * @brief 检查点是否在地图范围内
     */
    virtual bool isInside(double x, double y) const = 0;
};