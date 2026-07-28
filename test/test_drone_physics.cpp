#include <gtest/gtest.h>
#include "mavlink_nav_bridge/drone_physics.hpp"

using namespace mavlink_nav_bridge;

TEST(DronePhysics, RcNormalizeNeutral)
{
    EXPECT_DOUBLE_EQ(rc_normalize(1500), 0.0);
}

TEST(DronePhysics, RcNormalizeMax)
{
    EXPECT_DOUBLE_EQ(rc_normalize(2000), 1.0);
}

TEST(DronePhysics, RcNormalizeMin)
{
    EXPECT_DOUBLE_EQ(rc_normalize(1000), -1.0);
}

TEST(DronePhysics, DisarmedNoMovement)
{
    DroneState s;
    s.armed = false;
    s.z = 0.0;
    update_physics(s, 0.02);
    EXPECT_DOUBLE_EQ(s.z, 0.0);
    EXPECT_DOUBLE_EQ(s.x, 0.0);
}

TEST(DronePhysics, ThrottleCutAirborneDrops)
{
    DroneState s;
    s.armed = true;
    s.z = 5.0;
    s.rc[2] = 1000;  // throttle cut
    update_physics(s, 0.02);
    EXPECT_LT(s.vz, 0.0);  // falling
}

TEST(DronePhysics, ThrottleHoverStable)
{
    DroneState s;
    s.armed = true;
    s.z = 5.0;
    s.rc[2] = 1500;  // hover
    update_physics(s, 0.02);
    // Net accel = 0 at 1500, drag applied: vz stays near 0
    EXPECT_NEAR(s.vz, 0.0, 0.01);
}

TEST(DronePhysics, ThrottleAboveHoverClimbs)
{
    DroneState s;
    s.armed = true;
    s.z = 0.0;
    s.rc[2] = 2000;  // full throttle
    update_physics(s, 0.02);
    EXPECT_GT(s.vz, 0.0);
    EXPECT_GT(s.z, 0.0);
}

TEST(DronePhysics, PitchForwardMovesX)
{
    DroneState s;
    s.armed = true;
    s.z = 5.0;
    s.yaw = 0.0;
    s.rc[1] = 1400;  // pitch forward (below neutral → negate in physics = positive pitch)
    s.rc[2] = 1500;  // hover
    update_physics(s, 0.02);
    EXPECT_GT(s.vx, 0.0);  // moving forward in world X
}

TEST(DronePhysics, YawChangesHeading)
{
    DroneState s;
    s.armed = true;
    s.z = 5.0;
    s.rc[2] = 1500;
    s.rc[3] = 2000;  // full yaw right
    double yaw_before = s.yaw;
    update_physics(s, 0.02);
    EXPECT_GT(s.yaw, yaw_before);
}

TEST(DronePhysics, GroundClamp)
{
    DroneState s;
    s.armed = false;
    s.z = 0.1;
    s.vz = -10.0;  // falling hard
    update_physics(s, 0.02);
    EXPECT_GE(s.z, 0.0);
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
