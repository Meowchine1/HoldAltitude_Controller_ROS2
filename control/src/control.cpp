#include <rclcpp/rclcpp.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>
#include <future>

using namespace mavsdk;

class HoldAltitudeNode : public rclcpp::Node {
public:
    ~HoldAltitudeNode() {
    if (timer_) timer_->cancel();
    if (offboard_) offboard_->stop();
    if (action_) action_->disarm();
}
    HoldAltitudeNode(const std::string& connection_url, float target_altitude, double timer_frequency_hz)
    : Node("hold_altitude_node"),
      _connection_url(connection_url),
      _target_altitude(target_altitude)
    {
        using namespace std::chrono_literals;

        RCLCPP_INFO(this->get_logger(), "Connecting MAVSDK...");
        mavsdk_ = std::make_shared<Mavsdk>(Mavsdk::Configuration(1, 42, true));

        auto result = mavsdk_->add_any_connection(_connection_url);
        if (result != ConnectionResult::Success) {
            RCLCPP_ERROR(this->get_logger(), "Connection failed: %d", static_cast<int>(result));
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for system...");
        while (rclcpp::ok()) {
            for (auto& sys : mavsdk_->systems()) {
                if (sys->is_connected() && sys->has_autopilot()) {
                    system_ = sys;
                    break;
                }
            }
            if (system_) break;
            rclcpp::sleep_for(500ms);
        }

        telemetry_ = std::make_shared<Telemetry>(system_);
        action_ = std::make_shared<Action>(system_);
        offboard_ = std::make_shared<Offboard>(system_);

        telemetry_->set_rate_position(10.0);
        telemetry_->set_rate_altitude(10.0);

        RCLCPP_INFO(this->get_logger(), "Waiting for vehicle to be ready...");
        while (rclcpp::ok() && !telemetry_->health_all_ok()) {
            rclcpp::sleep_for(1s);
        }

        if (!telemetry_->armed()) {
            RCLCPP_INFO(this->get_logger(), "Arming...");
            if (action_->arm() != Action::Result::Success) {
                RCLCPP_ERROR(this->get_logger(), "Failed to arm");
                rclcpp::shutdown();
                return;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Taking off...");
        if (action_->takeoff() != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to take off");
            rclcpp::shutdown();
            return;
        }

        rclcpp::sleep_for(8s);

        RCLCPP_INFO(this->get_logger(), "Starting offboard control...");
        Offboard::VelocityNedYaw zero_velocity{};
        zero_velocity.north_m_s = 0.0f;
        zero_velocity.east_m_s = 0.0f;
        zero_velocity.down_m_s = 0.0f;
        zero_velocity.yaw_deg = 0.0f;

        offboard_->set_velocity_ned(zero_velocity);
        rclcpp::sleep_for(std::chrono::milliseconds(50)); 

        if (offboard_->start() != Offboard::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Offboard start failed");
            rclcpp::shutdown();
            return;
        } 
        
        dt_ = 1.0 / timer_frequency_hz;
        auto period = std::chrono::duration<double>(dt_);
        timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&HoldAltitudeNode::timer_callback, this)
    );
    }
private:
    std::string _connection_url;
    float _target_altitude;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<Mavsdk> mavsdk_;
    std::shared_ptr<System> system_;
    std::shared_ptr<Action> action_;
    std::shared_ptr<Telemetry> telemetry_;
    std::shared_ptr<Offboard> offboard_;
    
    float dt_ = 0.0f;
    float kp_ = 0.6f;
    float ki_ = 0.1f;
    float kd_ = 0.1f;

    float prev_error_ = 0.0f;
    float integral_ = 0.0f;
    float stable_counter_ = 0.0f;

    void timer_callback() {
        float current_alt = telemetry_->position().relative_altitude_m;
        float error = _target_altitude - current_alt;
        integral_ += error * dt_;
        float derivative = (error - prev_error_) / dt_;
        prev_error_ = error;

        float vel_down = clamp(-(kp_ * error + ki_ * integral_ + kd_ * derivative), -1.0f, 1.0f);

        Offboard::VelocityNedYaw velocity{};
        velocity.north_m_s = 0.0f;
        velocity.east_m_s = 0.0f;
        velocity.down_m_s = vel_down;
        velocity.yaw_deg = 0.0f;

        offboard_->set_velocity_ned(velocity);

        if (std::fabs(error) < 0.15f) {
            stable_counter_ += dt_;
        } else {
            stable_counter_ = 0; 
        }

        if (stable_counter_ > 10.0f) {
            RCLCPP_INFO(this->get_logger(), "Stable altitude reached, landing...");
            offboard_->stop();
            action_->land();
            timer_->cancel();
        }
       RCLCPP_INFO(this->get_logger(), " counter: %f", stable_counter_);
    }

float clamp(float val, float min_val, float max_val) {
    return std::max(min_val, std::min(val, max_val));
}
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    float target_height = 5.0;
    double timer_frequency_hz = 250.0;
    auto node = std::make_shared<HoldAltitudeNode>(argv[1], target_height, timer_frequency_hz);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
