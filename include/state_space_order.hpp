#pragma once

/**
 * @brief Ego vehicle input order
 *
 */
namespace EGO_INPUT
{
    static constexpr int STEER_ANGLE = 0;
    static constexpr int ACCEL = 1;
    static constexpr int DIM = 2;
} // namespace EGO_INPUT

/**
 * @brief Ego vehicle state-space order in frenet-serret coordinate which is used in MPC
 *
 */
namespace EGO_STATE_SPACE
{
    static constexpr int X_F = 0;
    static constexpr int Y_F = 1;
    static constexpr int YAW_F = 2;
    static constexpr int dY_F = 3;
    static constexpr int dYAW_F = 4;
    static constexpr int VEL = 5;
    static constexpr int DIM = 6;
} // namespace EGO_STATE_SPACE

/**
 * @brief Ego vehicle state-space order in global coordinate which is used in only simulator
 *
 */
namespace EGO_STATE_FOR_SIM
{
    static constexpr int X_G = 0;
    static constexpr int Y_G = 1;
    static constexpr int YAW_G = 2;
    static constexpr int TWIST_X = 3;
    static constexpr int TWIST_Y = 4;
    static constexpr int TWIST_YAW = 5;
    static constexpr int DIM = 6;
} // namespace EGO_STATE_FOR_SIM
