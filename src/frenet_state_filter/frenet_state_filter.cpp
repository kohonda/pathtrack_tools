#include "frenet_state_filter.hpp"

namespace pathtrack_tools
{
    FrenetStateFilter::FrenetStateFilter(const double &sampling_time) : sampling_time_{sampling_time}
    {
        prev_y_f_ = 0.0;
    }

    FrenetStateFilter::~FrenetStateFilter()
    {
    }

    void FrenetStateFilter::set_initial_pose(const FrenetCoordinate &pose_f, const Twist &twist)
    {

        prev_y_f_ = std::sqrt(twist.x * twist.x + twist.y * twist.y) * sin(pose_f.yaw_f);
    }

    // TODO : Kalman filter implementation
    double FrenetStateFilter::estimate_dy_f(const double &current_y_f)
    {
        // Most simple estimation
        const auto estimated_dy_f = (current_y_f - prev_y_f_) / sampling_time_;

        prev_y_f_ = estimated_dy_f;

        return estimated_dy_f;
    }

    double FrenetStateFilter::estimate_dy_f(const FrenetCoordinate &pose_f, const Twist &twist) const
    {
        // Vel x sin(yaw_f)
        const double estimated_dy_f = std::sqrt(twist.x * twist.x + twist.y * twist.y) * sin(pose_f.yaw_f);

        return estimated_dy_f;
    }

} // namespace pathtrack_tools
