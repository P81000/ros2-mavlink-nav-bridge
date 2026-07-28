#include <cmath>
#include <format>
#include <memory>
#include <numbers>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mavlink_nav_bridge/msg/rc_channels.hpp"
#include "mavlink_nav_bridge/msg/vehicle_command.hpp"
#include "mavlink_nav_bridge/msg/drone_status.hpp"
#include "mavlink_nav_bridge/drone_physics.hpp"

using VCmd = mavlink_nav_bridge::msg::VehicleCommand;
using namespace mavlink_nav_bridge;

class DroneSim : public rclcpp::Node
{
    public:
        DroneSim() : Node("drone_sim"), goto_active_(false), landing_(false), goto_x_(0), goto_y_(0), goto_z_(5.0) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        odom_pub_   = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/drone_marker", 10);
        status_pub_ = create_publisher<mavlink_nav_bridge::msg::DroneStatus>("/drone_status", 10);

        rc_sub_ = create_subscription<mavlink_nav_bridge::msg::RCChannels>(
                "/rc_channels", 10,
                [this](mavlink_nav_bridge::msg::RCChannels::SharedPtr msg) {
                    for (size_t i = 0; i < ch::COUNT; ++i)
                        state_.rc[i] = msg->channels[i];
                    goto_active_ = false;
                    landing_ = false;
                });

        cmd_sub_ = create_subscription<VCmd>("/vehicle_command", 10, std::bind(&DroneSim::on_command, this, std::placeholders::_1));

        timer_ = create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(&DroneSim::tick, this));

        RCLCPP_INFO(get_logger(), "drone_sim ready  pos=(0,0,0)  DISARMED");
    }

    private:
        void on_command(const VCmd::SharedPtr msg) {
            switch (msg->command) {
                case VCmd::ARM:
                    state_.armed = true;
                    RCLCPP_INFO(get_logger(), "ARMED — RC3>%u to take off, RC3=%u to hover", rc::NEUTRAL, rc::NEUTRAL);
                    break;
                case VCmd::DISARM:
                    state_.armed = false;
                    state_.vx = state_.vy = state_.vz = 0.0;
                    goto_active_ = false;
                    landing_ = false;
                    RCLCPP_INFO(get_logger(), "DISARMED");
                    break;
                case VCmd::LAND:
                    landing_ = true;
                    goto_active_ = false;
                    RCLCPP_INFO(get_logger(), "LAND — descending");
                    break;
                case VCmd::GOTO:
                    goto_x_ = msg->x;
                    goto_y_ = msg->y;
                    goto_z_ = msg->z;
                    goto_active_ = true;
                    landing_ = false;
                    RCLCPP_INFO(get_logger(), "GOTO (%.1f, %.1f, %.1f)", goto_x_, goto_y_, goto_z_);
                    break;
            }
        }

        void apply_goto() {
            double dx   = goto_x_ - state_.x;
            double dy   = goto_y_ - state_.y;
            double dz   = goto_z_ - state_.z;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < 0.3 && std::abs(dz) < 0.3) {
                goto_active_ = false;
                state_.rc[ch::ROLL]     = rc::NEUTRAL;
                state_.rc[ch::PITCH]    = rc::NEUTRAL;
                state_.rc[ch::THROTTLE] = rc::NEUTRAL;
                RCLCPP_INFO(get_logger(), "GOTO reached");
                return;
            }

            // Altitude: proportional throttle offset from hover
            double dz_norm = std::clamp(dz * 0.5, -1.0, 1.0);
            state_.rc[ch::THROTTLE] = static_cast<uint16_t>(
                    std::clamp(rc::NEUTRAL + dz_norm * (rc::MAX - rc::NEUTRAL),
                        static_cast<double>(rc::MIN),
                        static_cast<double>(rc::MAX)));

            // XY: rotate world target direction into body pitch/roll
            double cos_y  = std::cos(state_.yaw);
            double sin_y  = std::sin(state_.yaw);
            double mag    = std::min(dist * 0.8, 1.0);
            double ax_w   = (dist > 0.01) ? (dx / dist) * mag : 0.0;
            double ay_w   = (dist > 0.01) ? (dy / dist) * mag : 0.0;

            double pitch_body =  ax_w * cos_y + ay_w * sin_y;
            double roll_body  = -ax_w * sin_y + ay_w * cos_y;

            // pitch: positive = forward → RC2 below NEUTRAL
            state_.rc[ch::PITCH] = static_cast<uint16_t>(
                    std::clamp(rc::NEUTRAL - pitch_body * (rc::NEUTRAL - rc::MIN),
                        static_cast<double>(rc::MIN),
                        static_cast<double>(rc::MAX)));
            state_.rc[ch::ROLL] = static_cast<uint16_t>(
                    std::clamp(rc::NEUTRAL + roll_body * (rc::MAX - rc::NEUTRAL),
                        static_cast<double>(rc::MIN),
                        static_cast<double>(rc::MAX)));
        }

        void apply_land() {
            state_.rc[ch::THROTTLE] = rc::LAND_DESCENT;
            state_.rc[ch::PITCH]    = rc::NEUTRAL;
            state_.rc[ch::ROLL]     = rc::NEUTRAL;

            if (state_.z < 0.05) {
                landing_ = false;
                state_.armed = false;
                state_.vx = state_.vy = state_.vz = 0.0;
                RCLCPP_INFO(get_logger(), "LANDED — DISARMED");
            }
        }

        void tick() {
            constexpr double DT = 0.02;

            if (state_.armed) {
                if (goto_active_) apply_goto();
                else if (landing_) apply_land();
            }

            update_physics(state_, DT);

            auto now = get_clock()->now();
            publish_tf(now);
            publish_odom(now);
            publish_markers(now);
            publish_status();
        }

        void publish_tf(const rclcpp::Time & now) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp    = now;
            t.header.frame_id = "map";
            t.child_frame_id  = "base_link";
            t.transform.translation.x = state_.x;
            t.transform.translation.y = state_.y;
            t.transform.translation.z = state_.z;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, state_.yaw);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(t);
        }

        void publish_odom(const rclcpp::Time & now) {
            nav_msgs::msg::Odometry odom;
            odom.header.stamp    = now;
            odom.header.frame_id = "map";
            odom.child_frame_id  = "base_link";
            odom.pose.pose.position.x  = state_.x;
            odom.pose.pose.position.y  = state_.y;
            odom.pose.pose.position.z  = state_.z;
            odom.twist.twist.linear.x  = state_.vx;
            odom.twist.twist.linear.y  = state_.vy;
            odom.twist.twist.linear.z  = state_.vz;
            odom_pub_->publish(odom);
        }

        void publish_markers(const rclcpp::Time & now) {
            visualization_msgs::msg::MarkerArray arr;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, state_.yaw);

            visualization_msgs::msg::Marker body;
            body.header.stamp    = now;
            body.header.frame_id = "map";
            body.ns     = "drone";
            body.id     = 0;
            body.type   = visualization_msgs::msg::Marker::CUBE;
            body.action = visualization_msgs::msg::Marker::ADD;
            body.pose.position.x    = state_.x;
            body.pose.position.y    = state_.y;
            body.pose.position.z    = state_.z;
            body.pose.orientation.x = q.x();
            body.pose.orientation.y = q.y();
            body.pose.orientation.z = q.z();
            body.pose.orientation.w = q.w();
            body.scale.x = 0.5;
            body.scale.y = 0.5;
            body.scale.z = 0.12;
            body.color.r = state_.armed ? 0.1f : 0.9f;
            body.color.g = state_.armed ? 0.8f : 0.1f;
            body.color.b = 0.2f;
            body.color.a = 1.0f;
            arr.markers.push_back(body);

            visualization_msgs::msg::Marker label;
            label.header          = body.header;
            label.ns              = "drone";
            label.id              = 1;
            label.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label.action          = visualization_msgs::msg::Marker::ADD;
            label.pose.position.x = state_.x;
            label.pose.position.y = state_.y;
            label.pose.position.z = state_.z + 0.6;
            label.pose.orientation.w = 1.0;
            label.scale.z   = 0.35;
            label.color.r   = 1.0f;
            label.color.g   = 1.0f;
            label.color.b   = 1.0f;
            label.color.a   = 1.0f;
            label.text = std::format("{}  z={:.1f}m",
                    state_.armed ? "ARMED" : "DISARMED", state_.z);
            arr.markers.push_back(label);

            marker_pub_->publish(arr);
        }

        void publish_status() {
            mavlink_nav_bridge::msg::DroneStatus s;
            s.armed   = state_.armed;
            s.x       = state_.x;
            s.y       = state_.y;
            s.z       = state_.z;
            s.yaw_deg = state_.yaw * 180.0 / std::numbers::pi;
            s.vx      = state_.vx;
            s.vy      = state_.vy;
            s.vz      = state_.vz;
            status_pub_->publish(s);
        }

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Publisher<mavlink_nav_bridge::msg::DroneStatus>::SharedPtr status_pub_;
        rclcpp::Subscription<mavlink_nav_bridge::msg::RCChannels>::SharedPtr rc_sub_;
        rclcpp::Subscription<VCmd>::SharedPtr cmd_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        DroneState state_;

        bool   goto_active_;
        bool   landing_;
        double goto_x_, goto_y_, goto_z_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneSim>());
    rclcpp::shutdown();
    return 0;
}
