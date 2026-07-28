#include <format>
#include <iostream>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "mavlink_nav_bridge/msg/rc_channels.hpp"
#include "mavlink_nav_bridge/msg/vehicle_command.hpp"
#include "mavlink_nav_bridge/msg/drone_status.hpp"
#include "mavlink_nav_bridge/drone_physics.hpp"

using VCmd = mavlink_nav_bridge::msg::VehicleCommand;
using namespace mavlink_nav_bridge;

class GcsShell : public rclcpp::Node
{
    public:
        GcsShell() : Node("gcs_shell") {
            rc_pub_  = create_publisher<mavlink_nav_bridge::msg::RCChannels>("/rc_channels", 10);
            cmd_pub_ = create_publisher<VCmd>("/vehicle_command", 10);

            status_sub_ = create_subscription<mavlink_nav_bridge::msg::DroneStatus>(
                    "/drone_status", 10,
                    [this](mavlink_nav_bridge::msg::DroneStatus::SharedPtr msg) {
                    std::lock_guard<std::mutex> lk(status_mutex_);
                    last_status_ = *msg;
                    });

            std::ranges::fill(rc_.channels, rc::NEUTRAL);
            rc_.channels[ch::THROTTLE] = rc::MIN;

            timer_ = create_wall_timer(
                    std::chrono::milliseconds(50),
                    std::bind(&GcsShell::process_queue, this));

            stdin_thread_ = std::jthread([this](std::stop_token st) {
                    read_stdin(st);
                    });

            print_help();
            std::cout << "\n> " << std::flush;
        }

    private:
        void read_stdin(std::stop_token st) {
            std::string line;
            while (!st.stop_requested() && std::getline(std::cin, line)) {
                if (!line.empty()) {
                    std::lock_guard<std::mutex> lk(queue_mutex_);
                    cmd_queue_.push(line);
                }
                std::cout << "> " << std::flush;
            }
        }

        void process_queue() {
            std::queue<std::string> local;
            {
                std::lock_guard<std::mutex> lk(queue_mutex_);
                std::swap(local, cmd_queue_);
            }
            while (!local.empty()) {
                handle(local.front());
                local.pop();
            }
        }

        void handle(const std::string & line) {
            std::istringstream ss(line);
            std::string cmd;
            ss >> cmd;

            if (cmd == "arm") {
                send_vehicle_cmd(VCmd::ARM);
            } else if (cmd == "disarm") {
                rc_.channels[ch::THROTTLE] = rc::MIN;
                rc_pub_->publish(rc_);
                send_vehicle_cmd(VCmd::DISARM);
            } else if (cmd == "land") {
                send_vehicle_cmd(VCmd::LAND);
            } else if (cmd == "rc") {
                int      ch_in;
                uint16_t val;
                if (!(ss >> ch_in >> val)) {
                    std::cout << std::format("usage: rc <channel 1-{}> <value {}-{}>\n", ch::COUNT, rc::MIN, rc::MAX);
                    return;
                }
                if (ch_in < 1 || ch_in > static_cast<int>(ch::COUNT) ||
                        val < rc::MIN || val > rc::MAX)
                {
                    std::cout << std::format("channel must be 1-{}, value must be {}-{}\n", ch::COUNT, rc::MIN, rc::MAX);
                    return;
                }
                rc_.channels[ch_in - 1] = val;
                rc_pub_->publish(rc_);
                std::cout << std::format("RC{} = {}\n", ch_in, val);
            } else if (cmd == "goto") {
                double x, y, z = 5.0;
                if (!(ss >> x >> y)) {
                    std::cout << "usage: goto <x> <y> [z]\n";
                    return;
                }
                ss >> z;
                VCmd m;
                m.command = VCmd::GOTO;
                m.x = x; m.y = y; m.z = z;
                cmd_pub_->publish(m);
                std::cout << std::format("GOTO ({:.1f}, {:.1f}, {:.1f})\n", x, y, z);
            } else if (cmd == "status") {
                std::lock_guard<std::mutex> lk(status_mutex_);
                auto & s = last_status_;
                std::cout << std::format(
                        "armed={:<3}  pos=({:.2f}, {:.2f}, {:.2f})  yaw={:.1f}deg  vel=({:.2f}, {:.2f}, {:.2f})\n",
                        s.armed ? "YES" : "NO", s.x, s.y, s.z, s.yaw_deg, s.vx, s.vy, s.vz);
            } else if (cmd == "help") {
                print_help();
            } else {
                std::cout << "unknown: '" << cmd << "' — type 'help'\n";
            }
        }

        void send_vehicle_cmd(uint8_t cmd) {
            VCmd m;
            m.command = cmd;
            cmd_pub_->publish(m);
            std::cout << "[" << cmd_to_str(cmd) << "]\n";
        }

        static std::string_view cmd_to_str(uint8_t cmd) {
            switch (cmd) {
                case VCmd::ARM:    return "ARM";
                case VCmd::DISARM: return "DISARM";
                case VCmd::LAND:   return "LAND";
                case VCmd::GOTO:   return "GOTO";
                default:           return "UNKNOWN";
            }
        }

        void print_help() {
            std::cout <<
                "--------------------------------------------\n"
                "  MAVLink GCS Shell  —  ArduPilot Mode 2\n"
                "--------------------------------------------\n"
                "  arm                — arm motors\n"
                "  disarm             — disarm motors\n"
                "  land               — auto-land\n"
                "  rc <ch> <val>      — set RC channel (1-4, 1000-2000)\n"
                "    RC1 roll   RC2 pitch  RC3 throttle  RC4 yaw\n"
                "    1000=min / 1500=neutral / 2000=max\n"
                "    throttle: 1000=cut  1500=hover  >1500=climb\n"
                "    pitch:    <1500=forward  >1500=backward\n"
                "    yaw:      <1500=left     >1500=right\n"
                "  goto <x> <y> [z]  — fly to world position (m)\n"
                "  status             — print current drone state\n"
                "  help               — show this\n"
                "--------------------------------------------\n";
        }

        rclcpp::Publisher<mavlink_nav_bridge::msg::RCChannels>::SharedPtr rc_pub_;
        rclcpp::Publisher<VCmd>::SharedPtr cmd_pub_;
        rclcpp::Subscription<mavlink_nav_bridge::msg::DroneStatus>::SharedPtr status_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::jthread stdin_thread_;
        std::mutex   queue_mutex_;
        std::queue<std::string> cmd_queue_;

        std::mutex status_mutex_;
        mavlink_nav_bridge::msg::DroneStatus last_status_{};

        mavlink_nav_bridge::msg::RCChannels rc_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GcsShell>());
    rclcpp::shutdown();
    return 0;
}
