#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <ranges>

#define G 9.81

namespace mavlink_nav_bridge {
    // RC channel mapping (1-based user input → 0-based index):
    //   RC1 (idx 0) = Roll       1000=left   1500=neutral  2000=right
    //   RC2 (idx 1) = Pitch      1000=fwd    1500=neutral  2000=back  (ArduPilot: below neutral = nose down = forward)
    //   RC3 (idx 2) = Throttle   1000=cut    1500=hover    2000=full climb
    //   RC4 (idx 3) = Yaw        1000=left   1500=neutral  2000=right

    struct DroneState {
        double vx, vy, vz;
        double x, y, z;
        double yaw;  // radians
        uint16_t rc[4];
        bool armed;

        DroneState() : armed(false), x(0), y(0), z(0), vx(0), vy(0), vz(0), yaw(0) {
            std::ranges::fill(rc, uint16_t{1500});
            rc[2] = 1000;  // throttle starts at cut
        }
    };

    // Map RC value (1000-2000) to [-1.0, 1.0] relative to neutral (1500).
    inline double rc_normalize(uint16_t val) {
        return (static_cast<double>(val) - 1500.0) / 500.0;
    }

    constexpr double MAX_THRUST_NET     = 12.0;     // m/s² net above hover
    constexpr double MAX_PITCH_ACCEL    = 4.0;      // m/s² horizontal
    constexpr double MAX_YAW_RATE       = 1.5;      // rad/s
    constexpr double DRAG_H             = 0.94;     // per tick horizontal damping
    constexpr double DRAG_V             = 0.90;     // per tick vertical damping

    inline void update_physics(DroneState& s, double dt) {
        // Motor cut: not armed OR throttle at minimum
        if (!s.armed || s.rc[2] <= 1000) [[unlikely]] {
            if (s.z > 0.0) {
                s.vz -= G * dt;
            } else {
                s.vz = 0.0;
            }
            s.vx *= DRAG_H;
            s.vy *= DRAG_H;
            s.x += s.vx * dt;
            s.y += s.vy * dt;
            s.z += s.vz * dt;
            if (s.z < 0.0) s.z = 0.0;
            return;
        }

        // Throttle: 1500 = hover (net accel = 0), >1500 = climb, <1500 = sink
        double throttle_net = rc_normalize(s.rc[2]) * MAX_THRUST_NET;

        // Yaw (RC4, idx 3)
        s.yaw += rc_normalize(s.rc[3]) * MAX_YAW_RATE * dt;

        // Pitch (RC2, idx 1): below 1500 = forward in ArduPilot → negate normalize
        double pitch = -rc_normalize(s.rc[1]);

        // Roll (RC1, idx 0): above 1500 = right
        double roll = rc_normalize(s.rc[0]);

        // Body → world frame
        double cos_y = std::cos(s.yaw);
        double sin_y = std::sin(s.yaw);
        double ax = (pitch * cos_y - roll * sin_y) * MAX_PITCH_ACCEL;
        double ay = (pitch * sin_y + roll * cos_y) * MAX_PITCH_ACCEL;

        s.vx += ax * dt;
        s.vy += ay * dt;
        s.vz += throttle_net * dt;

        s.vx *= DRAG_H;
        s.vy *= DRAG_H;
        s.vz *= DRAG_V;

        s.x += s.vx * dt;
        s.y += s.vy * dt;
        s.z += s.vz * dt;

        if (s.z < 0.0) {
            s.z = 0.0;
            if (s.vz < 0.0) s.vz = 0.0;
        }
    }
}  // namespace mavlink_nav_bridge
