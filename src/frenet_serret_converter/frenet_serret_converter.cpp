#include "frenet_serret_converter.hpp"

namespace pathtrack_tools
{
FrenetSerretConverter::FrenetSerretConverter(/* args */)
{
    previous_nearest_index_           = 0;
    accumulated_x_f_to_nearest_index_ = 0.0;
    look_ahead_index_range_           = 100;  // TODO: あとから車速に合わせて可変にする
}

FrenetSerretConverter::~FrenetSerretConverter()
{
}

// reference_pathは毎制御周期ごとに変わることを意識
// autoware式の実装 : nearest waypointからの距離をy_f，nearest waypointまで繋いだ長さをx_fとする
// 本実装 :
// waypoint間を線形補間し，変換する．スパースなwaypointにも利用可能．ただし，waypintが重なっている場合には対応していない
FrenetCoordinate FrenetSerretConverter::global2frenet(const MPCCource& mpc_cource, const Pose& pose_g)
{
    // 1. Compute the index of the nearest neighbor. Search the entire look_ahead_index_range_range from the previous
    // index
    const int path_size                = mpc_cource.size();
    const int final_search_index       = std::min(path_size, previous_nearest_index_ + look_ahead_index_range_);
    const double current_nearest_index = search_nearest_point_index(previous_nearest_index_, final_search_index, mpc_cource, pose_g);

    // 2. Extract the two waypoints back and front the vehicle (in this case, there is no waypoint back and front)
    int forward_pt_index  = 0;
    int backward_pt_index = 0;
    if (current_nearest_index == 0)  // ego vehicle is located first way point
    {
        forward_pt_index  = 1;
        backward_pt_index = 0;
    }
    else if (current_nearest_index == path_size - 1)  // ego vehicle is located final waypoint
    {
        forward_pt_index  = path_size - 1;
        backward_pt_index = path_size - 2;
    }
    else
    {
        forward_pt_index  = extract_front_point(current_nearest_index, mpc_cource, pose_g);
        backward_pt_index = forward_pt_index - 1;
    }

    // 3. Draw a straight line connecting the two points of the front and back waypoints.
    // Ax+By+C=0
    const double A = mpc_cource.y[backward_pt_index] - mpc_cource.y[forward_pt_index];
    const double B = mpc_cource.x[forward_pt_index] - mpc_cource.x[backward_pt_index];
    const double C = -mpc_cource.x[backward_pt_index] * A - mpc_cource.y[backward_pt_index] * B;

    // 4. Caluculate y in frenet-serret coordinate system
    const double current_y_f = (A * pose_g.x + B * pose_g.y + C) / std::sqrt(A * A + B * B);

    // 5. Caluculate x in frenet-serret coordinate system
    // Calculate the total length of a straight line connecting two way points to the nearest waypoint.
    if (current_nearest_index != 0)
    {
        for (int i = previous_nearest_index_; i < current_nearest_index; i++)
        {
            const Eigen::Vector2d pt_i(mpc_cource.x[i], mpc_cource.y[i]);
            const Eigen::Vector2d pt_i_1(mpc_cource.x[i + 1], mpc_cource.y[i + 1]);
            accumulated_x_f_to_nearest_index_ += (pt_i - pt_i_1).norm();
        }
    }
    else  // vehicle is located at first waypoint
    {
        accumulated_x_f_to_nearest_index_ = 0.0;
    }
    // Add a diff length from nearest waypoint
    const double current_x_f = accumulated_x_f_to_nearest_index_ + delta_xf_from_nearest_pt(current_nearest_index, backward_pt_index, forward_pt_index, mpc_cource, pose_g);

    // 6. Caluculate yaw in frenet-serret coordinate system
    const double current_yaw_f = pose_g.yaw - std::atan2(-A, B);

    // 7. Take over the nearest_point_index for the next calculation.
    previous_nearest_index_ = current_nearest_index;

    const FrenetCoordinate pose_f(current_x_f, current_y_f, current_yaw_f);
    return pose_f;
}

Pose FrenetSerretConverter::frenet2global(const MPCCource& mpc_cource, const FrenetCoordinate& pose_f) const
{
    // find nerrest waypoint
    const std::vector<double> reference_x_f = mpc_cource.accumulated_path_length;

    const auto itr                   = std::lower_bound(reference_x_f.begin(), reference_x_f.end(), pose_f.x_f);
    const int lower_bound_index      = std::distance(reference_x_f.begin(), itr);
    const int prev_lower_bound_index = std::max(0, lower_bound_index - 1);

    const int nearest_index =
        (std::abs(reference_x_f.at(lower_bound_index) - pose_f.x_f) < std::abs(reference_x_f.at(prev_lower_bound_index) - pose_f.x_f)) ? lower_bound_index : prev_lower_bound_index;

    // Caluculation  (x_g^N, y_g^N) -> (x_g, y_g)
    const double PI = 3.141592653589793;
    const Eigen::Vector2d delta_vec_x_f((pose_f.x_f - reference_x_f.at(nearest_index)) * std::cos(mpc_cource.yaw.at(nearest_index)),
                                        (pose_f.x_f - reference_x_f.at(nearest_index)) * std::sin(mpc_cource.yaw.at(nearest_index)));
    const Eigen::Vector2d delta_vec_y_f(pose_f.y_f * std::cos(mpc_cource.yaw.at(nearest_index) + PI / 2.0), pose_f.y_f * std::sin(mpc_cource.yaw.at(nearest_index) + PI / 2.0));

    const Eigen::Vector2d nearest_point_g(mpc_cource.x.at(nearest_index), mpc_cource.y.at(nearest_index));

    const auto xy_g = nearest_point_g + delta_vec_x_f + delta_vec_y_f;

    // Calculate yaw_g

    const double yaw_g = mpc_cource.yaw.at(nearest_index) + pose_f.yaw_f;

    const Pose pose_g(xy_g(0), xy_g(1), 0.0, yaw_g, 0.0, 0.0);

    return pose_g;
}

// NOTE : (x,y)のみで最近傍点を探している．zやyawを用いていないので，crossing wayには対応していない
// この計算が一番時間がかかる
int FrenetSerretConverter::search_nearest_point_index(const int& first_point_index, const int& last_point_index, const MPCCource& mpc_cource, const Pose& pose_g) const
{
    if (mpc_cource.size() == 0)
    {
        std::cerr << "[MPCCource] Reference Cource size is zero!" << std::endl;
        return -1;
    }

    int nearest_index = -1;
    double min_dist   = std::numeric_limits<double>::max();

    for (int i = first_point_index; i < last_point_index; i++)
    {
        // std::hypotつかうよりも，Eigen使うよりもこの書き方が一番早い
        const double dist = std::sqrt((mpc_cource.x[i] - pose_g.x) * (mpc_cource.x[i] - pose_g.x) + (mpc_cource.y[i] - pose_g.y) * (mpc_cource.y[i] - pose_g.y));

        if (dist <= min_dist)
        {
            min_dist      = dist;
            nearest_index = i;
        }
    }

    return nearest_index;
}

int FrenetSerretConverter::extract_front_point(const int& nearest_point_index, const MPCCource& mpc_cource, const Pose& pose_g) const
{
    // Vector with the nearest neighbor as the origin
    const double nearest_x = mpc_cource.x[nearest_point_index];
    const double nearest_y = mpc_cource.y[nearest_point_index];
    const Eigen::Vector2d vec_nearest2pre_pt(mpc_cource.x[nearest_point_index - 1] - nearest_x, mpc_cource.y[nearest_point_index - 1] - nearest_y);
    const Eigen::Vector2d vec_nearest2next_pt(mpc_cource.x[nearest_point_index + 1] - nearest_x, mpc_cource.y[nearest_point_index + 1] - nearest_y);
    const Eigen::Vector2d vec_nearest2ego(pose_g.x - nearest_x, pose_g.y - nearest_y);

    int front_pt_index = 0;  // front point index of ego veghicle
    if (vec_nearest2ego.dot(vec_nearest2next_pt) < vec_nearest2ego.dot(vec_nearest2pre_pt))
    {
        // ego car is located at back of the nearest point
        front_pt_index = nearest_point_index;
    }
    else
    {
        // ego car is located at front of the nearest point
        front_pt_index = nearest_point_index + 1;
    }
    return front_pt_index;
}

double FrenetSerretConverter::delta_xf_from_nearest_pt(const int& nearest_point_index, const int& backward_pt_index, const int& forward_pt_index, const MPCCource& mpc_cource, const Pose& pose_g) const
{
    // Calculate the projection of the vector from the nearest point to the car to the back2front
    const Eigen::Vector2d vec_back2front(mpc_cource.x[forward_pt_index] - mpc_cource.x[backward_pt_index], mpc_cource.y[forward_pt_index] - mpc_cource.y[backward_pt_index]);
    const Eigen::Vector2d vec_nearest2ego(pose_g.x - mpc_cource.x[nearest_point_index], pose_g.y - mpc_cource.y[nearest_point_index]);
    const double delta_xf = vec_nearest2ego.dot(vec_back2front) / vec_back2front.norm();

    return delta_xf;
}

}  // namespace pathtrack_tools
