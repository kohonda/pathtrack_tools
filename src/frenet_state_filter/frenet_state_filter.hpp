#pragma once

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "FrenetCoordinate.hpp"
#include "Pose.hpp"
#include "Twist.hpp"

namespace pathtrack_tools
{
///
/// @class FrenetStateFilter
/// @brief
///

// TODO : 射影で求めるやつと，カルマンフィルタを実装する．mpc_simulatorにノイズ載せるやつがあるといいかもね
// TODO : 周辺他者のフィルタリングもあるといいね
class FrenetStateFilter
{
    public:
    /**
     * @brief Default constructor
     *
     */
    FrenetStateFilter(const double& sampling_time);

    /**
     * @brief Destroy the FrenetStateFilter object
     *
     */
    ~FrenetStateFilter();

    /**
     * @brief Set the initial pose object
     *
     * @param pose_f
     */
    void set_initial_pose(const FrenetCoordinate& pose_f, const Twist& twist);

    /**
     * @brief Estimate dy_f (df in frenet coordinate) from y_f which is converted from observed y_g
     *
     * @param y_f
     * @return double
     */
    double estimate_dy_f(const double& current_y_f);

    /**
     * @brief 参照経路方向の射影で決める
     *
     * @param current_pose_f
     * @param current_twist
     * @return double
     */
    double estimate_dy_f(const FrenetCoordinate& current_pose_f, const Twist& current_twist) const;

    private:
    const double sampling_time_;
    double prev_y_f_;
};

}  // namespace pathtrack_tools