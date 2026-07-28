#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <ranges>

namespace mavlink_nav_bridge {
    namespace rc {
        constexpr uint16_t MIN          = 1000;  // minimum PWM — throttle cut
        constexpr uint16_t NEUTRAL      = 1500;  // neutral / hover point
        constexpr uint16_t MAX          = 2000;  // maximum PWM
        constexpr uint16_t LAND_DESCENT = 1400;  // slightly below hover for controlled landing
    }

    namespace ch {
        constexpr size_t ROLL     = 0;
        constexpr size_t PITCH    = 1;  // below NEUTRAL = forward (ArduPilot Mode 2)
        constexpr size_t THROTTLE = 2;  // NEUTRAL = hover when airborne
        constexpr size_t YAW      = 3;
        constexpr size_t COUNT    = 4;
    }

    namespace physics {
        constexpr double GRAVITY         = 9.81;  // m/s²
        constexpr double MAX_PITCH_ACCEL = 4.0;   // m/s²
        constexpr double MAX_YAW_RATE    = 1.5;   // rad/s
        constexpr double DRAG_H          = 0.96;  // horizontal damping per tick (50 Hz)
        constexpr double DRAG_V          = 0.99;  // vertical damping per tick
    }

    struct DroneState {
        bool     armed;
        double   x, y, z;
        double   vx, vy, vz;
        double   yaw;           // radians
        uint16_t rc[ch::COUNT];

        DroneState()
            : armed(false), x(0), y(0), z(0), vx(0), vy(0), vz(0), yaw(0)
        {
            std::ranges::fill(rc, rc::NEUTRAL);
            rc[ch::THROTTLE] = rc::MIN;
        }
    };

    // Map RC value to [-1.0, 1.0] relative to NEUTRAL.
    inline double rc_normalize(uint16_t val) {
        return (static_cast<double>(val) - rc::NEUTRAL) /
            static_cast<double>(rc::MAX - rc::NEUTRAL);
    }

    // Thrust from throttle: MIN→0, NEUTRAL→gravity (hover), MAX→2*gravity.
    inline double throttle_to_thrust(uint16_t val) {
        double t = (static_cast<double>(val) - rc::MIN) /
            static_cast<double>(rc::MAX - rc::MIN);
        return t * 2.0 * physics::GRAVITY;
    }

    inline void update_physics(DroneState & s, double dt) {
        if (!s.armed || s.rc[ch::THROTTLE] <= rc::MIN) [[unlikely]] {
            if (s.z > 0.0) {
                s.vz -= physics::GRAVITY * dt;
            } else {
                s.vz = 0.0;
            }
            s.vx *= physics::DRAG_H;
            s.vy *= physics::DRAG_H;
            s.x  += s.vx * dt;
            s.y  += s.vy * dt;
            s.z  += s.vz * dt;
            if (s.z < 0.0) s.z = 0.0;
            return;
        }

        // Gravity always pulls down. Thrust counteracts it.
        // At rc::NEUTRAL: thrust == gravity → hover.
        // Below NEUTRAL: thrust < gravity → descend.
        // Above NEUTRAL: thrust > gravity → climb.
        s.vz -= physics::GRAVITY * dt;
        s.vz += throttle_to_thrust(s.rc[ch::THROTTLE]) * dt;

        // Yaw
        s.yaw += rc_normalize(s.rc[ch::YAW]) * physics::MAX_YAW_RATE * dt;

        // Pitch: below NEUTRAL = nose down = forward → negate
        double pitch = -rc_normalize(s.rc[ch::PITCH]);
        double roll  =  rc_normalize(s.rc[ch::ROLL]);

        // Body → world frame
        double cos_y = std::cos(s.yaw);
        double sin_y = std::sin(s.yaw);
        double ax = (pitch * cos_y - roll * sin_y) * physics::MAX_PITCH_ACCEL;
        double ay = (pitch * sin_y + roll * cos_y) * physics::MAX_PITCH_ACCEL;

        s.vx += ax * dt;
        s.vy += ay * dt;

        s.vx *= physics::DRAG_H;
        s.vy *= physics::DRAG_H;
        s.vz *= physics::DRAG_V;

        s.x += s.vx * dt;
        s.y += s.vy * dt;
        s.z += s.vz * dt;

        if (s.z < 0.0) {
            s.z = 0.0;
            if (s.vz < 0.0) s.vz = 0.0;
        }
    }
}  // namespace mavlink_nav_bridge
