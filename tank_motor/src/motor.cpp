#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lgpio.h>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

class Motor : public rclcpp::Node
{
public:
    Motor() : Node("motor_controller")
    {
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

        // Timer bezpieczeństwa - sprawdza co 100ms czy mamy połączenie
        watchdog_timer = this->create_wall_timer(
            100ms, std::bind(&Motor::WatchdogCheck, this));
   
        last_msg_time = this->now();

        RCLCPP_INFO(this->get_logger(), "Czołg gotowy. Watchdog aktywny.");
    }

    ~Motor()
    {
        StopRobot();
        lgGpiochipClose(h);
    }
private:
    int h;
    int left_pwm_pin, left_dir_pin, right_pwm_pin, right_dir_pin;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboardSub;
    rclcpp::TimerBase::SharedPtr watchdog_timer;
    rclcpp::Time last_msg_time;

    void StopRobot() {
        lgTxPwm(h, left_pwm_pin, 0, 0, 0, 0);
        lgTxPwm(h, right_pwm_pin, 0, 0, 0, 0);
        lgGpioWrite(h, left_dir_pin, 0);
        lgGpioWrite(h, right_dir_pin, 0);
    }

    void WatchdogCheck() {
        // Jeśli nie było wiadomości przez ponad 0.5 sekundy - STOP
        auto now = this->now();
        if ((now - last_msg_time).seconds() > 0.5) {
            StopRobot();
        }
    }

    void KeyBoardCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_msg_time = this->now(); // Odśwież czas ostatniej wiadomości

        float linear = msg->linear.x;
        float angular = msg->angular.z;

	float target_left  = linear + angular;
        float target_right = linear - angular;

        // Ograniczenie do zakresu -1.0 do 1.0
        target_left  = std::clamp(target_left, -1.0f, 1.0f);
        target_right = std::clamp(target_right, -1.0f, 1.0f);

        Send(left_pwm_pin, left_dir_pin, target_left);
        Send(right_pwm_pin, right_dir_pin, target_right);
    }

    void Send(int pwm_pin, int dir_pin, float value)
    {
        int forward = (value >= 0) ? 0 : 1;
        float duty_cycle = std::abs(value) * 100.0f;

        lgGpioWrite(h, dir_pin, forward);
        lgTxPwm(h, pwm_pin, 1000.0f, duty_cycle, 0, 0);
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
