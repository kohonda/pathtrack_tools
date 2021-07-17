#include <gtest/gtest.h>

#include "frenet_serret_converter.hpp"
#include "rapidcsv.h"
#include "StopWatch.hpp"

namespace pathtrack_tools
{
    class FrenetSerretConverterTest : public ::testing::Test
    {
    protected:
        virtual void SetUp()
        {
            frenet_serret_converter = new FrenetSerretConverter();
        }

        //テスト終了時に毎回実行されるFixture
        virtual void TearDown()
        {
            delete frenet_serret_converter;
        }

        FrenetSerretConverter *frenet_serret_converter;
    };

    MPCCource set_cource_test(const std::string &csv_path)
    {
        // csv read
        const rapidcsv::Document csv(csv_path);

        // get value from csv
        const std::vector<double> reference_x = csv.GetColumn<double>("reference.x");
        const std::vector<double> reference_y = csv.GetColumn<double>("reference.y");
        const std::vector<double> reference_speed = csv.GetColumn<double>("reference.speed");   // [m/s]
        const std::vector<double> drivable_delta_y_f = csv.GetColumn<double>("drivable_width"); // Mainly, cource width

        MPCCource mpc_cource;

        mpc_cource.resize(reference_x.size());
        mpc_cource.x = reference_x;
        mpc_cource.y = reference_y;
        mpc_cource.speed = reference_speed;
        mpc_cource.drivable_width = drivable_delta_y_f;

        return mpc_cource;
    }

    std::vector<double> calc_accumulated_path_length(const double &offset, const MPCCource &mpc_cource)
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
                const double delta_length = std::hypot(mpc_cource.x[i] - mpc_cource.x[i - 1], mpc_cource.y[i] - mpc_cource.y[i - 1]);
                accumulated_path_length[i] = accumulated_path_length[i - 1] + delta_length;
            }
        }

        return accumulated_path_length;
    }

    void set_accumulated_path_length(const double &offset, MPCCource *mpc_cource)
    {
        if (mpc_cource->size() == 0)
        {
            std::cerr << "[Warning] Reference Cource size is zero!" << std::endl;
        }

        mpc_cource->accumulated_path_length = calc_accumulated_path_length(offset, *mpc_cource);
    }

    void set_yaw(MPCCource *mpc_cource)
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

    TEST_F(FrenetSerretConverterTest, TestGlobal2Frenet_STRAIGHT)
    {
        const std::string long_straight_path = "../reference_path/step_path_01m.csv";
        MPCCource straight_path = set_cource_test(long_straight_path);
        set_accumulated_path_length(0.0, &straight_path);
        set_yaw(&straight_path);

        // StopWatch stop_watch;
        // stop_watch.lap();
        const Pose pose_g_minus(-1.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        const FrenetCoordinate pose_f_minus = frenet_serret_converter->global2frenet(straight_path, pose_g_minus);
        EXPECT_NEAR(-1.0, pose_f_minus.x_f, 1.0e-6);
        EXPECT_NEAR(1.0, pose_f_minus.y_f, 1.0e-6);
        EXPECT_NEAR(0.0, pose_f_minus.yaw_f, 1.0e-6);
        const Pose conv_pose_g_minus = frenet_serret_converter->frenet2global(straight_path, pose_f_minus);
        EXPECT_NEAR(pose_g_minus.x, conv_pose_g_minus.x, 1.0e-6);
        EXPECT_NEAR(pose_g_minus.y, conv_pose_g_minus.y, 1.0e-6);
        EXPECT_NEAR(pose_g_minus.yaw, conv_pose_g_minus.yaw, 1.0e-6);

        const Pose pose_g_0(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        const FrenetCoordinate pose_f_0 = frenet_serret_converter->global2frenet(straight_path, pose_g_0);
        EXPECT_NEAR(0.0, pose_f_0.x_f, 1.0e-6);
        EXPECT_NEAR(1.0, pose_f_0.y_f, 1.0e-6);
        EXPECT_NEAR(0.0, pose_f_0.yaw_f, 1.0e-6);
        const Pose conv_pose_g_0 = frenet_serret_converter->frenet2global(straight_path, pose_f_0);
        EXPECT_NEAR(pose_g_0.x, conv_pose_g_0.x, 1.0e-6);
        EXPECT_NEAR(pose_g_0.y, conv_pose_g_0.y, 1.0e-6);
        EXPECT_NEAR(pose_g_0.yaw, conv_pose_g_0.yaw, 1.0e-6);

        const Pose pose_g_1(0.9, 1.0, 0.0, 0.0, 0.0, 0.0);
        const FrenetCoordinate pose_f_1 = frenet_serret_converter->global2frenet(straight_path, pose_g_1);
        EXPECT_NEAR(0.9, pose_f_1.x_f, 1.0e-6);
        EXPECT_NEAR(1.0, pose_f_1.y_f, 1.0e-6);
        EXPECT_NEAR(0.0, pose_f_1.yaw_f, 1.0e-6);
        const Pose conv_pose_g_1 = frenet_serret_converter->frenet2global(straight_path, pose_f_1);
        EXPECT_NEAR(pose_g_1.x, conv_pose_g_1.x, 1.0e-6);
        EXPECT_NEAR(pose_g_1.y, conv_pose_g_1.y, 1.0e-6);
        EXPECT_NEAR(pose_g_1.yaw, conv_pose_g_1.yaw, 1.0e-6);

        const Pose pose_g_2(1.0, -0.5, 0.0, 0.0, 0.0, 0.0);
        const FrenetCoordinate pose_f_2 = frenet_serret_converter->global2frenet(straight_path, pose_g_2);
        EXPECT_NEAR(1.0, pose_f_2.x_f, 1.0e-6);
        EXPECT_NEAR(-0.5, pose_f_2.y_f, 1.0e-6);
        EXPECT_NEAR(0.0, pose_f_2.yaw_f, 1.0e-6);
        const Pose conv_pose_g_2 = frenet_serret_converter->frenet2global(straight_path, pose_f_2);
        EXPECT_NEAR(pose_g_2.x, conv_pose_g_2.x, 1.0e-6);
        EXPECT_NEAR(pose_g_2.y, conv_pose_g_2.y, 1.0e-6);
        EXPECT_NEAR(pose_g_2.yaw, conv_pose_g_2.yaw, 1.0e-6);

        const Pose pose_g_3(1.1, 0.0, 0.0, 0.0, 0.0, 0.0);
        const FrenetCoordinate pose_f_3 = frenet_serret_converter->global2frenet(straight_path, pose_g_3);
        EXPECT_NEAR(1.1, pose_f_3.x_f, 1.0e-6);
        EXPECT_NEAR(0.0, pose_f_3.y_f, 1.0e-6);
        EXPECT_NEAR(0.0, pose_f_3.yaw_f, 1.0e-6);
        const Pose conv_pose_g_3 = frenet_serret_converter->frenet2global(straight_path, pose_f_3);
        EXPECT_NEAR(pose_g_3.x, conv_pose_g_3.x, 1.0e-6);
        EXPECT_NEAR(pose_g_3.y, conv_pose_g_3.y, 1.0e-6);
        EXPECT_NEAR(pose_g_3.yaw, conv_pose_g_3.yaw, 1.0e-6);

        const Pose pose_g_4(2.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        const FrenetCoordinate pose_f_4 = frenet_serret_converter->global2frenet(straight_path, pose_g_4);
        EXPECT_NEAR(2.0, pose_f_4.x_f, 1.0e-6);
        EXPECT_NEAR(1.0, pose_f_4.y_f, 1.0e-6);
        EXPECT_NEAR(0.0, pose_f_4.yaw_f, 1.0e-6);
        const Pose conv_pose_g_4 = frenet_serret_converter->frenet2global(straight_path, pose_f_4);
        EXPECT_NEAR(pose_g_4.x, conv_pose_g_4.x, 1.0e-6);
        EXPECT_NEAR(pose_g_4.y, conv_pose_g_4.y, 1.0e-6);
        EXPECT_NEAR(pose_g_4.yaw, conv_pose_g_4.yaw, 1.0e-6);

        const Pose pose_g_5(3.0, -1.0, 0.0, 0.0, 0.0, 0.0);
        const FrenetCoordinate pose_f_5 = frenet_serret_converter->global2frenet(straight_path, pose_g_5);
        EXPECT_NEAR(3.0, pose_f_5.x_f, 1.0e-6);
        EXPECT_NEAR(-1.0, pose_f_5.y_f, 1.0e-6);
        EXPECT_NEAR(0.0, pose_f_5.yaw_f, 1.0e-6);
        const Pose conv_pose_g_5 = frenet_serret_converter->frenet2global(straight_path, pose_f_5);
        EXPECT_NEAR(pose_g_5.x, conv_pose_g_5.x, 1.0e-6);
        EXPECT_NEAR(pose_g_5.y, conv_pose_g_5.y, 1.0e-6);
        EXPECT_NEAR(pose_g_5.yaw, conv_pose_g_5.yaw, 1.0e-6);

        const Pose pose_g_over(4.0, -1.0, 0.0, 0.0, 0.0, 0.0);
        const FrenetCoordinate pose_f_over = frenet_serret_converter->global2frenet(straight_path, pose_g_over);
        EXPECT_NEAR(4.0, pose_f_over.x_f, 1.0e-6);
        EXPECT_NEAR(-1.0, pose_f_over.y_f, 1.0e-6);
        EXPECT_NEAR(0.0, pose_f_over.yaw_f, 1.0e-6);
        const Pose conv_pose_g_over = frenet_serret_converter->frenet2global(straight_path, pose_f_over);
        EXPECT_NEAR(pose_g_over.x, conv_pose_g_over.x, 1.0e-6);
        EXPECT_NEAR(pose_g_over.y, conv_pose_g_over.y, 1.0e-6);
        EXPECT_NEAR(pose_g_over.yaw, conv_pose_g_over.yaw, 1.0e-6);

        // std::cout << "calc time (original struct)[msec] :" << stop_watch.lap() << std::endl;
    }

    TEST_F(FrenetSerretConverterTest, TestGlobal2Frenet_STRAIGHT_2)
    {
        const std::string long_straight_path = "../reference_path/straight_path2.csv";
        MPCCource straight_path = set_cource_test(long_straight_path);
        set_accumulated_path_length(0.0, &straight_path);
        set_yaw(&straight_path);

        const Pose pose_g_1(5.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        const FrenetCoordinate pose_f_1 = frenet_serret_converter->global2frenet(straight_path, pose_g_1);
        EXPECT_NEAR(0.0, pose_f_1.x_f, 1.0e-6);
        EXPECT_NEAR(-5.0, pose_f_1.y_f, 1.0e-6);
        EXPECT_NEAR(-1.57079632679, pose_f_1.yaw_f, 1.0e-6);
        const Pose conv_pose_g_1 = frenet_serret_converter->frenet2global(straight_path, pose_f_1);
        EXPECT_NEAR(pose_g_1.x, conv_pose_g_1.x, 1.0e-6);
        EXPECT_NEAR(pose_g_1.y, conv_pose_g_1.y, 1.0e-6);
        EXPECT_NEAR(pose_g_1.yaw, conv_pose_g_1.yaw, 1.0e-6);

        const Pose pose_g_2(-1.0, 5.0, 0.0, 0.0, 0.0, 0.0);
        const FrenetCoordinate pose_f_2 = frenet_serret_converter->global2frenet(straight_path, pose_g_2);
        EXPECT_NEAR(4.0, pose_f_2.x_f, 1.0e-6);
        EXPECT_NEAR(1.0, pose_f_2.y_f, 1.0e-6);
        EXPECT_NEAR(-1.57079632679, pose_f_2.yaw_f, 1.0e-6);
        const Pose conv_pose_g_2 = frenet_serret_converter->frenet2global(straight_path, pose_f_2);
        EXPECT_NEAR(pose_g_2.x, conv_pose_g_2.x, 1.0e-6);
        EXPECT_NEAR(pose_g_2.y, conv_pose_g_2.y, 1.0e-6);
        EXPECT_NEAR(pose_g_2.yaw, conv_pose_g_2.yaw, 1.0e-6);
    }

} // namespace pathtrack_tools