#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lgpio.h>
#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class Motor : public rclcpp::Node
{
public:
    Motor() : Node("motor_controller")
    {
        declare_parameter("pwm_frequency_hz", 5000.0);
        declare_parameter("watchdog_timeout_sec", 0.8);
        declare_parameter("min_duty_percent", 15.0);

        pwm_frequency_ = get_parameter("pwm_frequency_hz").as_double();
        watchdog_timeout_ = get_parameter("watchdog_timeout_sec").as_double();
        min_duty_percent_ = get_parameter("min_duty_percent").as_double();

        left_pwm_pin = 17;
        left_dir_pin = 27;
        right_pwm_pin = 22;
        right_dir_pin = 23;

        h = lgGpiochipOpen(4);

        if (h < 0) {
            RCLCPP_ERROR(this->get_logger(), "Błąd lgpio: %d", h);
            exit(1);
        }

        lgGpioClaimOutput(h, 0, left_pwm_pin, 0);
        lgGpioClaimOutput(h, 0, left_dir_pin, 0);
        lgGpioClaimOutput(h, 0, right_pwm_pin, 0);
        lgGpioClaimOutput(h, 0, right_dir_pin, 0);

        keyboardSub = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&Motor::KeyBoardCallback, this, std::placeholders::_1)
        );

        watchdog_timer = this->create_wall_timer(
            100ms, std::bind(&Motor::WatchdogCheck, this));

        last_msg_time = this->now();
        StopRobot();

        RCLCPP_INFO(this->get_logger(),
                    "Czołg gotowy. PWM=%.0f Hz, watchdog=%.1fs",
                    pwm_frequency_, watchdog_timeout_);
    }

    ~Motor()
    {
        StopRobot();
        lgGpiochipClose(h);
    }

private:
    int h;
    int left_pwm_pin, left_dir_pin, right_pwm_pin, right_dir_pin;
    double pwm_frequency_;
    double watchdog_timeout_;
    double min_duty_percent_;

    float last_left_ = 0.0f;
    float last_right_ = 0.0f;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboardSub;
    rclcpp::TimerBase::SharedPtr watchdog_timer;
    rclcpp::Time last_msg_time;

    void StopRobot()
    {
        lgTxPwm(h, left_pwm_pin, static_cast<float>(pwm_frequency_), 0.0f, 0, 0);
        lgTxPwm(h, right_pwm_pin, static_cast<float>(pwm_frequency_), 0.0f, 0, 0);
        lgGpioWrite(h, left_dir_pin, 0);
        lgGpioWrite(h, right_dir_pin, 0);
        last_left_ = 0.0f;
        last_right_ = 0.0f;
    }

    void WatchdogCheck()
    {
        auto now = this->now();
        if ((now - last_msg_time).seconds() > watchdog_timeout_) {
            if (last_left_ != 0.0f || last_right_ != 0.0f) {
                StopRobot();
            }
        }
    }

    void KeyBoardCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_msg_time = this->now();

        float linear = msg->linear.x;
        float angular = msg->angular.z;

        float target_left  = linear + angular;
        float target_right = linear - angular;

        target_left  = std::clamp(target_left, -1.0f, 1.0f);
        target_right = std::clamp(target_right, -1.0f, 1.0f);

        if (std::abs(target_left - last_left_) < 0.001f
            && std::abs(target_right - last_right_) < 0.001f) {
            return;
        }

        if (std::abs(target_left) < 0.01f && std::abs(target_right) < 0.01f) {
            StopRobot();
            return;
        }

        Send(left_pwm_pin, left_dir_pin, target_left);
        Send(right_pwm_pin, right_dir_pin, target_right);

        last_left_ = target_left;
        last_right_ = target_right;
    }

    void Send(int pwm_pin, int dir_pin, float value)
    {
        int forward = (value >= 0) ? 0 : 1;
        float duty_cycle = std::abs(value) * 100.0f;

        if (duty_cycle > 0.0f && duty_cycle < min_duty_percent_) {
            duty_cycle = static_cast<float>(min_duty_percent_);
        }

        lgGpioWrite(h, dir_pin, forward);
        lgTxPwm(h, pwm_pin, static_cast<float>(pwm_frequency_), duty_cycle, 0, 0);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Motor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
