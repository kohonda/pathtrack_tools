#include <gtest/gtest.h>
#include "StopWatch.hpp"
#include "course_manager.hpp"

namespace pathtrack_tools
{
    class CourseManagerTest : public ::testing::Test
    {
    protected:
        virtual void SetUp()
        {
            course_manager = new CourseManager();
        }

        virtual void TearDown()
        {
            delete course_manager;
        }

        CourseManager *course_manager;
    };

    TEST_F(CourseManagerTest, TestStraightLane_CSV)
    {
        const std::string valid_cource = "/home/honda/PathTrack_Tools/reference_path/straight_lane.csv";
        // StopWatch stop_watch;
        // stop_watch.lap();
        course_manager->set_cource_from_csv(valid_cource);

        ASSERT_EQ(100, course_manager->get_path_size());
        // accumulated length
        EXPECT_EQ(9.9, course_manager->get_mpc_cource().accumulated_path_length.at(99));
        EXPECT_EQ(0.0, course_manager->get_mpc_cource().accumulated_path_length.at(0));
        // curvature without interporated
        EXPECT_EQ(0.0, course_manager->get_mpc_cource().curvature.at(20));
        EXPECT_EQ(0.0, course_manager->get_mpc_cource().curvature.at(2));
        EXPECT_EQ(0.0, course_manager->get_mpc_cource().curvature.at(99));
        EXPECT_EQ(0.0, course_manager->get_mpc_cource().curvature.at(95));
        EXPECT_EQ(0.0, course_manager->get_mpc_cource().curvature.at(0));
        EXPECT_EQ(0.0, course_manager->get_mpc_cource().curvature.at(10));
        // curvature/speed/drivable_width with interporated
        EXPECT_NEAR(0.0, course_manager->get_curvature(0.0), 1.0e-9);
        EXPECT_NEAR(11.111, course_manager->get_speed(0.0), 1.0e-3);
        EXPECT_NEAR(11.111, course_manager->get_speed(0.0), 1.0e-3);
        EXPECT_NEAR(2.0, course_manager->get_drivable_width(0.0), 1.0e-3);
        EXPECT_NEAR(0.0, course_manager->get_curvature(1.0), 1.0e-9);
        EXPECT_NEAR(11.111, course_manager->get_speed(1.0), 1.0e-3);
        EXPECT_NEAR(2.0, course_manager->get_drivable_width(1.0), 1.0e-3);
        EXPECT_NEAR(0.0, course_manager->get_curvature(5.0), 1.0e-9);
        EXPECT_NEAR(11.111, course_manager->get_speed(5.0), 1.0e-3);
        EXPECT_NEAR(2.0, course_manager->get_drivable_width(5.0), 1.0e-3);
        EXPECT_NEAR(0.0, course_manager->get_curvature(9.9), 1.0e-9);
        EXPECT_NEAR(11.111, course_manager->get_speed(9.9), 1.0e-3);
        EXPECT_NEAR(2.0, course_manager->get_drivable_width(9.9), 1.0e-3);
        EXPECT_NEAR(0.0, course_manager->get_curvature(11.0), 1.0e-9);
        EXPECT_NEAR(11.111, course_manager->get_speed(11.0), 1.0e-3);
        EXPECT_NEAR(2.0, course_manager->get_drivable_width(11.0), 1.0e-3);
        // std::cout << "calc time [msec] :" << stop_watch.lap() << std::endl;
    }

    TEST_F(CourseManagerTest, CircuitLane_CSV)
    {
        const std::string valid_cource = "/home/honda/PathTrack_Tools/reference_path/circuit_lane.csv";
        course_manager->set_cource_from_csv(valid_cource);

        ASSERT_EQ(201, course_manager->get_path_size());
        // accumulated length
        EXPECT_EQ(0.0, course_manager->get_mpc_cource().accumulated_path_length.at(0));
        EXPECT_NEAR(15.707963267948966, course_manager->get_mpc_cource().accumulated_path_length.at(100), 1.0e-2);
        EXPECT_NEAR(31.41592653589793, course_manager->get_mpc_cource().accumulated_path_length.at(200), 1.0e-2);

        // curvature withot interporated
        EXPECT_NEAR(0.2, course_manager->get_mpc_cource().curvature.at(0), 1.0e-9);
        EXPECT_NEAR(0.2, course_manager->get_mpc_cource().curvature.at(5), 1.0e-9);
        EXPECT_NEAR(0.2, course_manager->get_mpc_cource().curvature.at(10), 1.0e-9);
        EXPECT_NEAR(0.2, course_manager->get_mpc_cource().curvature.at(30), 1.0e-9);
        EXPECT_NEAR(0.2, course_manager->get_mpc_cource().curvature.at(50), 1.0e-9);
        EXPECT_NEAR(0.2, course_manager->get_mpc_cource().curvature.at(100), 1.0e-9);
        EXPECT_NEAR(0.2, course_manager->get_mpc_cource().curvature.at(195), 1.0e-9);
        EXPECT_NEAR(0.2, course_manager->get_mpc_cource().curvature.at(196), 1.0e-9);
        EXPECT_NEAR(0.2, course_manager->get_mpc_cource().curvature.at(200), 1.0e-9);

        // curvature interporated
        EXPECT_NEAR(0.2, course_manager->get_curvature(1.0e-9), 1.0e-9);
        EXPECT_NEAR(11.111, course_manager->get_speed(1.0e-9), 1.0e-3);
        EXPECT_NEAR(2.0, course_manager->get_drivable_width(1.0e-9), 1.0e-3);
        EXPECT_NEAR(0.2, course_manager->get_curvature(1.0), 1.0e-9);
        EXPECT_NEAR(11.111, course_manager->get_speed(1.0), 1.0e-3);
        EXPECT_NEAR(2.0, course_manager->get_drivable_width(1.0), 1.0e-3);
        EXPECT_NEAR(0.2, course_manager->get_curvature(15.0), 1.0e-9);
        EXPECT_NEAR(11.111, course_manager->get_speed(15.0), 1.0e-3);
        EXPECT_NEAR(2.0, course_manager->get_drivable_width(15.0), 1.0e-3);
        EXPECT_NEAR(0.2, course_manager->get_curvature(31.41592653589793), 1.0e-9);
        EXPECT_NEAR(11.111, course_manager->get_speed(31.41592653589793), 1.0e-3);
        EXPECT_NEAR(2.0, course_manager->get_drivable_width(31.41592653589793), 1.0e-3);
        EXPECT_NEAR(0.2, course_manager->get_curvature(33.0), 1.0e-9);
        EXPECT_NEAR(11.111, course_manager->get_speed(33.0), 1.0e-3);
        EXPECT_NEAR(2.0, course_manager->get_drivable_width(33.0), 1.0e-3);
    }

    TEST_F(CourseManagerTest, NotConst_CSV)
    {
        const std::string valid_cource = "/home/honda/PathTrack_Tools/reference_path/not_const_value.csv";
        course_manager->set_cource_from_csv(valid_cource);

        // curvature interporated
        EXPECT_NEAR(0.0, course_manager->get_curvature(1.0e-9), 1.0e-9);
        EXPECT_NEAR(0.0, course_manager->get_speed(1.0e-9), 1.0e-3);
        EXPECT_NEAR(2.0, course_manager->get_drivable_width(1.0e-9), 1.0e-3);
        EXPECT_NEAR(0.0, course_manager->get_curvature(5.0), 1.0e-9);
        EXPECT_NEAR(50.0, course_manager->get_speed(5.0), 1.0e-3);
        EXPECT_NEAR(6.0, course_manager->get_drivable_width(5.0), 1.0e-3);
        EXPECT_NEAR(0.0, course_manager->get_curvature(2.0), 1.0e-9);
        EXPECT_NEAR(20.0, course_manager->get_speed(2.0), 1.0e-3);
        EXPECT_NEAR(3.6, course_manager->get_drivable_width(2.0), 1.0e-3);
        EXPECT_NEAR(0.0, course_manager->get_curvature(16.0), 1.0e-9);
        EXPECT_NEAR(55.0, course_manager->get_speed(15.0), 1.0e-3);
        EXPECT_NEAR(5.5, course_manager->get_drivable_width(15.0), 1.0e-3);
    }

    TEST_F(CourseManagerTest, NotConst_CSV_2)
    {
        const std::string valid_cource = "/home/honda/PathTrack_Tools/reference_path/not_const_value_2.csv";
        course_manager->set_cource_from_csv(valid_cource);
        // curvature interporated
        EXPECT_NEAR(1.0e-9, course_manager->get_speed(1.0e-9), 1.0e-9);

        EXPECT_NEAR(1.1, course_manager->get_speed(1.1), 1.0e-9);
        EXPECT_NEAR(2.0, course_manager->get_speed(2.0), 1.0e-9);

        EXPECT_NEAR(0.0, course_manager->get_speed(0.0), 1.0e-9);
    }

} // namespace pathtrack_tools