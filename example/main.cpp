
#include "StopWatch.hpp"
#include "course_manager.hpp"
#include "frenet_serret_converter.hpp"
#include "vehicle_dynamics_simulator.hpp"
#include "Twist.hpp"
#include "frenet_state_filter.hpp"

int main()
{

    // Observed info of ego vehicle
    Pose ego_pose_global; // ego pose in global coordinates
    Twist ego_twist;

    // Initialize pose and twist of ego car
    ego_pose_global.x = 0.0;
    ego_pose_global.y = 0.0;
    ego_pose_global.z = 0.0;
    ego_pose_global.roll = 0.0;
    ego_pose_global.pitch = 0.0;
    ego_pose_global.yaw = 0.0;
    ego_twist.x = 0.0; // Vehicle Longitudinal speed
    ego_twist.y = 0.0;
    ego_twist.yaw = 0.0;

    // Observed info of crossing pedestrian
    FrenetCoordinate ped_pose_frenet;
    Twist ped_twist;

    // Initialize pose and twist of crossing pedestrain
    ped_pose_frenet.x_f = 10;
    ped_pose_frenet.y_f = -2.0;
    ped_pose_frenet.yaw_f = 0.0;
    ped_twist.x = 0.0;
    ped_twist.y = 1.0;

    // Set Driving course
    pathtrack_tools::CourseManager course_manager;
    course_manager.set_course_from_csv("../reference_path/sinwave_course.csv");

    std::function<double(double)> course_curvature = [&course_manager](const double &x_f) { return course_manager.get_curvature(x_f); };
    std::function<double(double)> course_speed = [&course_manager](const double &x_f) { return course_manager.get_speed(x_f); };
    std::function<double(double)> drivable_width = [&course_manager](const double &x_f) { return course_manager.get_drivable_width(x_f); };

    // Frenet Serret Conveter
    pathtrack_tools::FrenetSerretConverter frenet_serret_converter;

    const double sampling_time = 0.01;
    // Frenet state estimate dy_f
    pathtrack_tools::FrenetStateFilter frenet_state_filter(sampling_time);

    // Reproduce the motion of the control target by numerical integration for simulation
    // cgmres::CGMRESSimulator simulator;
    pathtrack_tools::VehicleDynamicsSimulator vehicle_dynamics_simulator(sampling_time);

    // State and Control input

    double control_input_vec[EGO_INPUT::DIM]; // calculated control input (tire_angle, accel)

    // Stop Watch
    StopWatch stop_watch;

    const double start_time = 0.0;
    const double end_time = 40.0;

    // Simulation loop
    std::cout << "Start Simulation" << std::endl;
    for (double current_time = start_time; current_time < end_time; current_time += sampling_time)
    {
        // stop watch start
        stop_watch.lap();

        // coordinate convert
        const FrenetCoordinate ego_pose_frenet = frenet_serret_converter.global2frenet(course_manager.get_mpc_course(), ego_pose_global);

        const auto estimated_dy_f = frenet_state_filter.estimate_dy_f(ego_pose_frenet.y_f);

        // ================================
        // Calculate control input
        // ================================

        // stop watch end
        const double calculation_time = stop_watch.lap(); // calculation time at one control interval [msec]

        // Pose & Twist update by simulator
        const auto [updated_ego_pose_global, updated_ego_twist] = vehicle_dynamics_simulator.update_ego_state(current_time, ego_pose_global, ego_twist, control_input_vec, sampling_time);
    }

    std::cout << "End Simulation" << std::endl;

    return 0;
}
