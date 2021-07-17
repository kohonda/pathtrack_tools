#include "cource_manager.hpp"

namespace pathtrack_tools
{
CourceManager::CourceManager(/* args */) : curvature_smoothing_num_{ 10 }, max_curvature_change_rate_{ 0.1 }, hash_resolution_{ 0.01 }, redudant_xf_{ 5.0 }
{
    nearest_index_        = 0;
    nearest_ratio_        = 1.0;
    second_nearest_index_ = 0;
    second_nearest_ratio_ = 0.0;
    current_pose_x_f_     = 0.0;
    hash_xf2index_        = { { 0.0, 0 } };
}

CourceManager::~CourceManager()
{
}

int CourceManager::get_path_size() const
{
    return mpc_cource_.size();
}

MPCCource CourceManager::get_mpc_cource() const
{
    return mpc_cource_;
}

// Read the course information from the csv and set them to a member variable.
void CourceManager::set_cource_from_csv(const std::string& csv_path)
{
    // csv read
    const rapidcsv::Document csv(csv_path);

    // get value from csv
    const std::vector<double> reference_x        = csv.GetColumn<double>("reference.x");
    const std::vector<double> reference_y        = csv.GetColumn<double>("reference.y");
    const std::vector<double> reference_speed    = csv.GetColumn<double>("reference.speed");  // [m/s]
    const std::vector<double> drivable_delta_y_f = csv.GetColumn<double>("drivable_width");   // Mainly, cource width

    // Check all vectors have the same size and the size is not zero
    try
    {
        if (reference_x.size() == 0)
        {
            throw std::range_error("Cource data size is zero!");
        }
        else if (reference_x.size() != reference_y.size() || reference_x.size() != reference_speed.size() || reference_x.size() != drivable_delta_y_f.size())
        {
            throw std::range_error("All cource data size are not same!");
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    // set value to rerference_cource_ with index
    mpc_cource_.resize(reference_x.size());
    mpc_cource_.x              = reference_x;
    mpc_cource_.y              = reference_y;
    mpc_cource_.speed          = reference_speed;
    mpc_cource_.drivable_width = drivable_delta_y_f;

    // set accumulated path length, which is nearly equal to x_f
    // offset = 0.0, which must be set previous xf when mpc_cource is given sequentially.
    set_accumulated_path_length(0.0, &mpc_cource_);

    // set curvature, which is nearly equal to x_f
    set_path_curvature(curvature_smoothing_num_, &mpc_cource_);

    // set path yaw_g for using global2frenet
    set_path_yaw(&mpc_cource_);

    // Smooth the reference path when its curvature change rate is large.
    filtering_path_curvature(mpc_cource_.accumulated_path_length, &mpc_cource_.curvature);

    // set lookup table covert from xf to nearest index
    set_hash_xf2index(mpc_cource_, &hash_xf2index_);
}

// offset = 更新されたmpc_courceの最初のx,yのx_fの値
void CourceManager::set_accumulated_path_length(const double& offset, MPCCource* mpc_cource)
{
    if (mpc_cource->size() == 0)
    {
        std::cerr << "[Warning] Reference Cource size is zero!" << std::endl;
    }

    mpc_cource->accumulated_path_length = calc_accumulated_path_length(offset, *mpc_cource);
}

// それぞれのrefernce point間を線分で補間して，reference pointまでの距離を計算する
std::vector<double> CourceManager::calc_accumulated_path_length(const double& offset, const MPCCource& mpc_cource) const
{
    std::vector<double> accumulated_path_length(mpc_cource.x.size());
    for (size_t i = 0; i < accumulated_path_length.size(); i++)
    {
        // calculate accumulated path length, which is nearly equal to x_f
        if (i == 0)
        {
            accumulated_path_length[0] = offset;
        }
        else
        {
            const double delta_length  = std::hypot(mpc_cource.x[i] - mpc_cource.x[i - 1], mpc_cource.y[i] - mpc_cource.y[i - 1]);
            accumulated_path_length[i] = accumulated_path_length[i - 1] + delta_length;
        }
    }

    return accumulated_path_length;
}

void CourceManager::set_path_curvature(const int& smoothing_num, MPCCource* mpc_cource)
{
    if (mpc_cource->size() == 0)
    {
        std::cerr << "[Warning] Reference Cource size is zero!" << std::endl;
    }
    mpc_cource->curvature = calc_path_curvature(smoothing_num, *mpc_cource);
}

// Now, Only used for frenet2global coordinate
void CourceManager::set_path_yaw(MPCCource* mpc_cource)
{
    for (int i = 0; i < mpc_cource->size() - 2; i++)
    {
        Eigen::Vector2d p(mpc_cource->x.at(i), mpc_cource->y.at(i));
        Eigen::Vector2d p_next(mpc_cource->x.at(i + 1), mpc_cource->y.at(i + 1));

        const auto delta_vec = p_next - p;

        mpc_cource->yaw.at(i) = std::atan2(delta_vec(1), delta_vec(0));
    }
    mpc_cource->yaw.at(mpc_cource->size() - 1) = mpc_cource->yaw.at(mpc_cource->size() - 2);
}

// それぞれのreference pointにおける曲率を計算する．
// 滑らかな曲率にするために数点分前後の3点をもちいて円でフィッティング
// TODO : 曲率の変化率の最大値を頭打ちするようなフィルタリングしたいね -> そしてステップを走れるようにしたい
std::vector<double> CourceManager::calc_path_curvature(const int& smoothing_num, const MPCCource& mpc_cource) const
{
    const int path_size = mpc_cource.x.size();
    std::vector<double> curvature_vec(path_size);

    const int max_smoothing_num = std::floor(0.5 * (path_size - 1));
    const int L                 = std::min(max_smoothing_num, smoothing_num);

    // reference pathの端のほう以外のcurvatureの計算
    // 反時計回りのとき, 曲率 ρ > 0 ,
    // 　時計周りのとき, 曲率 ρ < 0 ,
    // パスが直線のとき, 曲率 ρ = 0
    for (int i = L; i < path_size - L; i++)
    {
        const Eigen::Vector2d p1(mpc_cource.x[i - L], mpc_cource.y[i - L]);
        const Eigen::Vector2d p2(mpc_cource.x[i], mpc_cource.y[i]);
        const Eigen::Vector2d p3(mpc_cource.x[i + L], mpc_cource.y[i + L]);
        const double denominator = std::max((p1 - p2).norm() * (p2 - p3).norm() * (p3 - p1).norm(), 0.0001);

        const double curvature = 2.0 * ((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])) / denominator;
        curvature_vec[i]       = curvature;
    }

    // 端の方のcurvatureの計算
    for (int i = 0; i < std::min(path_size, L); i++)
    {
        curvature_vec[i]                 = curvature_vec[std::min(L, path_size - 1)];
        curvature_vec[path_size - i - 1] = curvature_vec[std::max(path_size - L - 1, 0)];
    }

    return curvature_vec;
}

double CourceManager::calc_distance(const std::array<double, 2>& p1, const std::array<double, 2>& p2) const
{
    return std::hypot(p1[0] - p2[0], p1[1] - p2[1]);
}

// Saturate the rate of curvature change when curvature_change_rate > max_curvature_change_rate
void CourceManager::filtering_path_curvature(const std::vector<double>& accumulated_path_length, std::vector<double>* path_curvature)
{
    bool is_filterd = false;

    // return (+1 or -1 or 0) according to the sign of x value
    auto sign = [](const double& x) { return (x > 0) - (x < 0); };

    for (size_t i = 1; i < path_curvature->size(); i++)
    {
        const double delta_xf              = accumulated_path_length[i] - accumulated_path_length[i - 1];
        const double delta_curvature       = path_curvature->at(i) - path_curvature->at(i - 1);
        const double curvature_change_rate = delta_curvature / std::max(0.00001, delta_xf);

        if (std::abs(curvature_change_rate) > max_curvature_change_rate_)
        {
            path_curvature->at(i) = path_curvature->at(i - 1) + sign(delta_curvature) * max_curvature_change_rate_ * delta_xf;

            is_filterd = true;
        }
    }
    if (is_filterd)
    {
        std::cout << "Filterd mpc_cource curvature because path curvature change rate is too big." << std::endl;
    }
}

void CourceManager::set_hash_xf2index(const MPCCource& mpc_cource, std::unordered_map<double, int>* hash_xf2index)
{
    hash_xf2index->clear();
    const double start_xf = round_resolution(mpc_cource.accumulated_path_length.front() - redudant_xf_);
    const double end_xf   = round_resolution(mpc_cource.accumulated_path_length.back());

    int last_nearest_index = 0;
    for (double xf = start_xf; xf < end_xf; xf += hash_resolution_)
    {
        const int nearest_index = search_nearest_index(mpc_cource, xf, last_nearest_index);
        last_nearest_index      = nearest_index;
        const double round_xf   = round_resolution(xf);
        hash_xf2index->emplace(round_xf, nearest_index);
    }
}

double CourceManager::round_resolution(const double& raw_x_f) const
{
    // returen the x_f value rounded by the resolution
    const double round_x_f = hash_resolution_ * std::round(raw_x_f / hash_resolution_);
    return round_x_f;
}

double CourceManager::get_curvature(const double& pose_x_f)
{
    // 同一予測ステップ内で何度も同じ計算を避けるために
    // 同一予測ステップで初めてcurvature or speed or
    // drivable_widthをgetするときは内分率と前後のリファレンスポイントをセットする
    if (pose_x_f != current_pose_x_f_)
    {
        current_pose_x_f_ = pose_x_f;
        // nearest indexを計算
        // nearest_index_      = search_nearest_index(mpc_cource_, pose_x_f, last_nearest_index_);
        // last_nearest_index_ = nearest_index_;
        nearest_index_ = lookup_xf_to_nearest_index(hash_xf2index_, pose_x_f);

        // Calculate internal ratio of pose_x_f between nearest and second_nearest
        set_internal_ratio(mpc_cource_, pose_x_f, nearest_index_);
    }

    const double curvature_interporated = linear_interporate(mpc_cource_.curvature, nearest_index_, nearest_ratio_, second_nearest_index_, second_nearest_ratio_);

    // 曲率はinterporateしないほうが調子がいい??
    // const double curvature_interporated = mpc_cource_.curvature[nearest_index_];
    return curvature_interporated;
}

double CourceManager::get_speed(const double& pose_x_f)
{
    // 同一予測ステップ内で何度も同じ計算を避けるために
    // 同一予測ステップで初めてcurvature or speed or
    // drivable_widthをgetするときは内分率と前後のリファレンスポイントをセットする
    if (pose_x_f != current_pose_x_f_)
    {
        current_pose_x_f_ = pose_x_f;
        // nearest indexを計算
        // nearest_index_      = search_nearest_index(mpc_cource_, pose_x_f, last_nearest_index_);
        // last_nearest_index_ = nearest_index_;
        nearest_index_ = lookup_xf_to_nearest_index(hash_xf2index_, pose_x_f);

        // Calculate internal ratio of pose_x_f between nearest and second_nearest
        set_internal_ratio(mpc_cource_, pose_x_f, nearest_index_);
    }

    const double reference_speed_interporated = linear_interporate(mpc_cource_.speed, nearest_index_, nearest_ratio_, second_nearest_index_, second_nearest_ratio_);
    return reference_speed_interporated;
}

double CourceManager::get_drivable_width(const double& pose_x_f)
{
    // 同一予測ステップ内で何度も同じ計算を避けるために
    // 同一予測ステップで初めてcurvature or speed or
    // drivable_widthをgetするときは内分率と前後のリファレンスポイントをセットする
    if (pose_x_f != current_pose_x_f_)
    {
        current_pose_x_f_ = pose_x_f;
        // nearest indexを計算
        // nearest_index_      = search_nearest_index(mpc_cource_, pose_x_f, last_nearest_index_);
        // last_nearest_index_ = nearest_index_;
        nearest_index_ = lookup_xf_to_nearest_index(hash_xf2index_, pose_x_f);

        // Calculate internal ratio of pose_x_f between nearest and second_nearest
        set_internal_ratio(mpc_cource_, pose_x_f, nearest_index_);
    }

    const double drivable_width_interporated = linear_interporate(mpc_cource_.drivable_width, nearest_index_, nearest_ratio_, second_nearest_index_, second_nearest_ratio_);
    return drivable_width_interporated;
}

// pose_x_f : 車両のフレネー座標におけるx
// フレネー座標のx_fを比較してnearest_indexを探す
int CourceManager::search_nearest_index(const MPCCource& mpc_cource, const double& pose_x_f, const int& last_nearest_index) const noexcept
{
    // NOTE : Not consider z and yaw
    // もしリファレンスパスがcrossingすることがあればzとかyawを考慮したほうがいい
    // 例えば，yawとreference_yawを比較して，おおきければ近くても無視するといった処理
    int tmp_nearest_index = last_nearest_index;
    // 直前のnearest indexを初期値として再急降下法っぽい処理
    for (int i = 0; i < mpc_cource.size(); i++)
    {
        const int prev_index = std::max(0, tmp_nearest_index - 1);
        const int next_index = std::min(tmp_nearest_index + 1, mpc_cource.size() - 1);

        const auto dist_to_prev = std::abs(pose_x_f - mpc_cource.accumulated_path_length[prev_index]);
        const auto dist_to_last = std::abs(pose_x_f - mpc_cource.accumulated_path_length[tmp_nearest_index]);
        const auto dist_to_next = std::abs(pose_x_f - mpc_cource.accumulated_path_length[next_index]);

        if (dist_to_last <= dist_to_prev && dist_to_last <= dist_to_next)
        {
            const int nearest_index = tmp_nearest_index;
            return nearest_index;
        }
        else
        {
            if (dist_to_prev < dist_to_next)
            {
                tmp_nearest_index = prev_index;
            }
            else if (dist_to_prev > dist_to_next)
            {
                tmp_nearest_index = next_index;
            }
        }
    }

    return -1;
}

// 車両のposeとpath pointの各点の距離を比較してnearest indexを探す
// Not used now
int CourceManager::search_nearest_index(const MPCCource& mpc_cource, const Pose& pose) const
{
    if (mpc_cource.size() == 0)
    {
        std::cerr << "[Warning] Reference Cource size is zero!" << std::endl;
        return -1;
    }

    int nearest_index = -1;
    double min_dist   = std::numeric_limits<double>::max();

    // NOTE : Not consider z and yaw
    // もしリファレンスパスがcrossingすることがあればzとかyawを考慮したほうがいい
    // 例えば，yawとreference_yawを比較して，おおきければ近くても無視するといった処理

    for (int i = 0; i < mpc_cource.size(); i++)
    {
        const auto dx   = mpc_cource.x[i] - pose.x;
        const auto dy   = mpc_cource.y[i] - pose.y;
        const auto dist = dx * dx + dy * dy;

        if (dist < min_dist)
        {
            min_dist      = dist;
            nearest_index = i;
        }
    }

    return nearest_index;
}

int CourceManager::lookup_xf_to_nearest_index(const std::unordered_map<double, int>& hash_xf2index, const double& pose_x_f) const
{
    const double round_x_f = round_resolution(pose_x_f);

    int nearest_index = 0;

    try
    {
        nearest_index = hash_xf2index.at(round_x_f);
    }
    catch (const std::out_of_range& ex)
    {
        std::cerr << "[Warning] Out of range of Reference cource. Plsease set longer mpc_cource. " << std::endl;
        nearest_index = mpc_cource_.size() - 1;
    }

    return nearest_index;
}

void CourceManager::set_internal_ratio(const MPCCource& mpc_cource, const double& pose_x_f, const int& nearest_index)
{
    // calculate second nearest index based difference from x_f
    // second nearest==nearest となるパターン : (1) nearestの直上にある (2) nearest_index == final or first indexかつprev or next == nearestのとき(つまり，path上には車なし)
    const int prev_index    = std::max(0, nearest_index - 1);
    const int next_index    = std::min(nearest_index + 1, mpc_cource.size() - 1);
    const auto diff_nearest = pose_x_f - mpc_cource_.accumulated_path_length[nearest_index];
    if (diff_nearest < 0)
    {
        // vehicle is between nearest and prev point
        second_nearest_index_ = prev_index;
    }
    else if (diff_nearest > 0)
    {
        // vehicle is between nearest and next point
        second_nearest_index_ = next_index;
    }
    else if (diff_nearest == 0)
    {
        // veihcle is on the nearest point
        second_nearest_index_ = nearest_index_;
    }

    // Calculate internal ratio of pose_x_f between nearest and second_nearest
    const double dist_nearest_and_second = std::abs(mpc_cource_.accumulated_path_length[second_nearest_index_] - mpc_cource_.accumulated_path_length[nearest_index_]);
    // nearest と second nearestが近すぎるときのnanを防ぐy
    if (dist_nearest_and_second < 1.0e-5)
    {
        nearest_ratio_        = 0.0;
        second_nearest_ratio_ = 1.0;
    }
    else
    {
        const double dist_to_nearest        = std::abs(mpc_cource_.accumulated_path_length[nearest_index] - pose_x_f);
        const double dist_to_second_nearest = std::abs(mpc_cource_.accumulated_path_length[second_nearest_index_] - pose_x_f);
        nearest_ratio_                      = dist_to_nearest / dist_nearest_and_second;
        second_nearest_ratio_               = dist_to_second_nearest / dist_nearest_and_second;
    }
}

double CourceManager::linear_interporate(const std::vector<double>& reference_vec, const int& nearest_index, const double& nearest_ratio, const int& second_nearest_index,
                                         const double& second_nearest_ratio) const noexcept
{
    return second_nearest_ratio * reference_vec[nearest_index] + nearest_ratio * reference_vec[second_nearest_index];
}
}  // namespace pathtrack_tools
// mespace pathtrac_tools
