#include <gtest/gtest.h>
#include "mavlink_nav_bridge/drone_physics.hpp"

using namespace mavlink_nav_bridge;

TEST(DronePhysics, RcNormalizeNeutral)
{
    EXPECT_DOUBLE_EQ(rc_normalize(rc::NEUTRAL), 0.0);
}

TEST(DronePhysics, RcNormalizeMax)
{
    EXPECT_DOUBLE_EQ(rc_normalize(rc::MAX), 1.0);
}

TEST(DronePhysics, RcNormalizeMin)
{
    EXPECT_DOUBLE_EQ(rc_normalize(rc::MIN), -1.0);
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
    s.rc[ch::THROTTLE] = rc::MIN;  // motor cut
    update_physics(s, 0.02);
    EXPECT_LT(s.vz, 0.0);  // falling
}

TEST(DronePhysics, ThrottleAtNeutralHoversAirborne)
{
    DroneState s;
    s.armed = true;
    s.z = 5.0;
    s.rc[ch::THROTTLE] = rc::NEUTRAL;  // thrust == gravity → hover
    update_physics(s, 0.02);
    // After one tick: gravity - thrust = 0, drag applied → vz stays near 0
    EXPECT_NEAR(s.vz, 0.0, 0.05);
}

TEST(DronePhysics, ThrottleAboveNeutralClimbs)
{
    DroneState s;
    s.armed = true;
    s.z = 0.0;
    s.rc[ch::THROTTLE] = rc::MAX;  // full throttle
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
    s.rc[ch::THROTTLE] = rc::NEUTRAL;
    s.rc[ch::PITCH]    = rc::MIN + 100;  // below neutral = forward
    update_physics(s, 0.02);
    EXPECT_GT(s.vx, 0.0);  // moving forward in world X
}

TEST(DronePhysics, YawChangesHeading)
{
    DroneState s;
    s.armed = true;
    s.z = 5.0;
    s.rc[ch::THROTTLE] = rc::NEUTRAL;
    s.rc[ch::YAW]      = rc::MAX;  // full yaw right
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
