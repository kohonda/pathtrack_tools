#include "vehicle_dynamics_simulator.hpp"

namespace pathtrack_tools
{
    VehicleDynamicsSimulator::VehicleDynamicsSimulator(const double &sampling_time)
    {
        initialize_input_queue(sampling_time);
    }

    VehicleDynamicsSimulator::~VehicleDynamicsSimulator()
    {
    }

    std::pair<Pose, Twist> VehicleDynamicsSimulator::update_ego_state(const double current_time, const Pose &current_ego_pose_global, const Twist &current_ego_twist, const double *control_input_vec,
                                                                      const double sampling_time)
    {
        const std::array<double, EGO_STATE_FOR_SIM::DIM> current_ego_vehicle_state = ego_struct_to_state(current_ego_pose_global, current_ego_twist);

        auto state_func = [this](auto current_time, auto x, auto u) {
            const double speed_threshold_dynamics = 1.0; // [m/s] because dbm is less accurate at low vehicle speeds and cannot be used when vehicle speed is negative
            if (x[EGO_STATE_FOR_SIM::TWIST_X] < speed_threshold_dynamics)
            {
                return this->fx_kbm_with_delay(current_time, x, u);
            }
            else
            {
                return this->fx_dbm_with_delay(current_time, x, u);
            }
        };

        const std::array<double, EGO_STATE_FOR_SIM::DIM> updated_ego_vehicle_state = runge_kutta_gill(current_time, current_ego_vehicle_state, control_input_vec, sampling_time, state_func);

        const auto [updated_ego_pose, updated_ego_twist] = ego_state_to_struct(updated_ego_vehicle_state);

        return {updated_ego_pose, updated_ego_twist};
    }

    std::pair<FrenetCoordinate, Twist> VehicleDynamicsSimulator::update_pedestrian_state(const FrenetCoordinate &current_pose, const Twist &current_twist, const double sampling_time) const
    {
        FrenetCoordinate updated_pose;
        Twist updated_twist;

        // constant velocity walking
        updated_twist = current_twist;

        // update pose
        updated_pose.x_f = current_pose.x_f + updated_twist.x * sampling_time;
        updated_pose.y_f = current_pose.y_f + updated_twist.y * sampling_time;

        return {updated_pose, updated_twist};
    }

    std::array<double, EGO_STATE_FOR_SIM::DIM> VehicleDynamicsSimulator::ego_struct_to_state(const Pose &ego_pose, const Twist &ego_twist) const
    {
        std::array<double, EGO_STATE_FOR_SIM::DIM> ego_state;

        ego_state.at(EGO_STATE_FOR_SIM::X_G) = ego_pose.x;
        ego_state.at(EGO_STATE_FOR_SIM::Y_G) = ego_pose.y;
        ego_state.at(EGO_STATE_FOR_SIM::YAW_G) = ego_pose.yaw;
        ego_state.at(EGO_STATE_FOR_SIM::TWIST_X) = ego_twist.x;
        ego_state.at(EGO_STATE_FOR_SIM::TWIST_Y) = ego_twist.y;
        ego_state.at(EGO_STATE_FOR_SIM::TWIST_YAW) = ego_twist.yaw;

        return ego_state;
    }

    std::pair<Pose, Twist> VehicleDynamicsSimulator::ego_state_to_struct(const std::array<double, EGO_STATE_FOR_SIM::DIM> &ego_state) const
    {
        Pose ego_pose;
        Twist ego_twist;

        ego_pose.x = ego_state.at(EGO_STATE_FOR_SIM::X_G);
        ego_pose.y = ego_state.at(EGO_STATE_FOR_SIM::Y_G);
        ego_pose.yaw = ego_state.at(EGO_STATE_FOR_SIM::YAW_G);
        ego_twist.x = ego_state.at(EGO_STATE_FOR_SIM::TWIST_X);
        ego_twist.y = ego_state.at(EGO_STATE_FOR_SIM::TWIST_Y);
        ego_twist.yaw = ego_state.at(EGO_STATE_FOR_SIM::TWIST_YAW);

        return {ego_pose, ego_twist};
    }

    void VehicleDynamicsSimulator::initialize_input_queue(const double &sampling_time)
    {
        size_t accel_input_queue_size = static_cast<size_t>(round(accel_delay_time_ / sampling_time));
        for (size_t i = 0; i < accel_input_queue_size; i++)
        {
            accel_input_queue_.push(0.0);
        }

        size_t angle_input_queue_size = static_cast<size_t>(round(angle_delay_time_ / sampling_time));
        for (size_t i = 0; i < angle_input_queue_size; i++)
        {
            angle_input_queue_.push(0.0);
        }
    }

    double VehicleDynamicsSimulator::delay_accel_input(const double &raw_accel_input)
    {
        accel_input_queue_.push(raw_accel_input);
        const double delayed_accel_input = accel_input_queue_.front();
        accel_input_queue_.pop();

        return delayed_accel_input;
    }

    double VehicleDynamicsSimulator::delay_angle_input(const double &raw_angle_input)
    {
        angle_input_queue_.push(raw_angle_input);
        const double delayed_angle_input = angle_input_queue_.front();
        angle_input_queue_.pop();

        return delayed_angle_input;
    }

    std::array<double, EGO_STATE_FOR_SIM::DIM> VehicleDynamicsSimulator::fx_kbm_without_delay(const double current_time, const std::array<double, EGO_STATE_FOR_SIM::DIM> &x, const double *u) const
    {
        std::array<double, EGO_STATE_FOR_SIM::DIM> dx;

        const double curvature = 0.0;

        double x0 = vehicle_lf_ + vehicle_lr_;
        double x1 = std::tan(u[EGO_INPUT::STEER_ANGLE]);
        double x2 = x1 / x0;
        double x3 = x[EGO_STATE_FOR_SIM::YAW_G] + std::atan2(vehicle_lr_ * x2, 1.0);
        double x4 = x[EGO_STATE_FOR_SIM::TWIST_X] * std::cos(x3) / (-curvature * x[EGO_STATE_FOR_SIM::Y_G] + 1);

        dx[EGO_STATE_FOR_SIM::Y_G] = x[EGO_STATE_FOR_SIM::TWIST_X] * std::sin(x3);
        dx[EGO_STATE_FOR_SIM::YAW_G] = -curvature * x4 + x2 * x[EGO_STATE_FOR_SIM::TWIST_X] / std::sqrt(pow(vehicle_lr_, 2) * pow(x1, 2) / pow(x0, 2) + 1);
        dx[EGO_STATE_FOR_SIM::X_G] = x4;
        dx[EGO_STATE_FOR_SIM::TWIST_X] = u[EGO_INPUT::ACCEL];
        dx[EGO_STATE_FOR_SIM::TWIST_Y] = 0.0;
        dx[EGO_STATE_FOR_SIM::TWIST_YAW] = 0.0;

        return dx;
    }

    std::array<double, EGO_STATE_FOR_SIM::DIM> VehicleDynamicsSimulator::fx_kbm_with_delay(const double current_time, const std::array<double, EGO_STATE_FOR_SIM::DIM> &x, const double *u)
    {
        const double raw_angle_input = u[EGO_INPUT::STEER_ANGLE];
        const double raw_accel_input = u[EGO_INPUT::ACCEL];

        const double delayed_angle_input = delay_angle_input(raw_angle_input);
        const double delayed_accel_input = delay_accel_input(raw_accel_input);

        const double delayed_u[EGO_INPUT::DIM] = {delayed_angle_input, delayed_accel_input};

        const std::array<double, EGO_STATE_FOR_SIM::DIM> dx = fx_kbm_without_delay(current_time, x, delayed_u);

        return dx;
    }

    std::array<double, EGO_STATE_FOR_SIM::DIM> VehicleDynamicsSimulator::fx_dbm_without_delay(const double current_time, const std::array<double, EGO_STATE_FOR_SIM::DIM> &x, const double *u) const
    {
        const double d_x_g = x[EGO_STATE_FOR_SIM::TWIST_X] * std::cos(x[EGO_STATE_FOR_SIM::YAW_G]) - x[EGO_STATE_FOR_SIM::TWIST_Y] * std::sin(x[EGO_STATE_FOR_SIM::YAW_G]);
        const double d_y_g = x[EGO_STATE_FOR_SIM::TWIST_X] * std::sin(x[EGO_STATE_FOR_SIM::YAW_G]) + x[EGO_STATE_FOR_SIM::TWIST_Y] * std::cos(x[EGO_STATE_FOR_SIM::YAW_G]);
        const double d_yaw_g = x[EGO_STATE_FOR_SIM::TWIST_YAW];

        const double F_fy = -2 * vehicle_kf_ *
                            std::atan2(((x[EGO_STATE_FOR_SIM::TWIST_Y] + vehicle_lf_ * x[EGO_STATE_FOR_SIM::TWIST_YAW]) / std::max(x[EGO_STATE_FOR_SIM::TWIST_X], 0.001) - u[EGO_INPUT::STEER_ANGLE]), 1.0);
        const double F_ry = -2 * vehicle_kr_ * std::atan2(((x[EGO_STATE_FOR_SIM::TWIST_Y] - vehicle_lr_ * x[EGO_STATE_FOR_SIM::TWIST_YAW]) / std::max(x[EGO_STATE_FOR_SIM::TWIST_X], 0.001)), 1.0);

        const double d_twist_x = u[EGO_INPUT::ACCEL] - F_fy * std::sin(u[EGO_INPUT::STEER_ANGLE]) / vehicle_mass_ + x[EGO_STATE_FOR_SIM::TWIST_Y] * x[EGO_STATE_FOR_SIM::TWIST_YAW];
        const double d_twist_y = F_ry / vehicle_mass_ + F_fy * std::cos(u[EGO_INPUT::STEER_ANGLE]) / vehicle_mass_ - x[EGO_STATE_FOR_SIM::TWIST_X] * x[EGO_STATE_FOR_SIM::TWIST_YAW];
        const double d_twist_yaw = (F_fy * vehicle_lf_ * std::cos(u[EGO_INPUT::STEER_ANGLE]) - F_ry * vehicle_lr_) / vehicle_inertia_;

        // return d/dt[x_g, y_g, yaw_g, twist.x, twist.y, twist.yaw]
        std::array<double, EGO_STATE_FOR_SIM::DIM> dx = {d_x_g, d_y_g, d_yaw_g, d_twist_x, d_twist_y, d_twist_yaw};

        return dx;
    }

    std::array<double, EGO_STATE_FOR_SIM::DIM> VehicleDynamicsSimulator::fx_dbm_with_delay(const double current_time, const std::array<double, EGO_STATE_FOR_SIM::DIM> &x, const double *u)
    {
        const double raw_angle_input = u[EGO_INPUT::STEER_ANGLE];
        const double raw_accel_input = u[EGO_INPUT::ACCEL];

        const double delayed_angle_input = delay_angle_input(raw_angle_input);
        const double delayed_accel_input = delay_accel_input(raw_accel_input);

        const double delayed_u[EGO_INPUT::DIM] = {delayed_angle_input, delayed_accel_input};

        const std::array<double, EGO_STATE_FOR_SIM::DIM> dx = fx_dbm_without_delay(current_time, x, delayed_u);

        return dx;
    }

} // namespace pathtrack_tools
