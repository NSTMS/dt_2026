#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <lgpio.h>
#include <vector>
#include <cmath>
#include <algorithm>

class ActuatorController : public rclcpp::Node {
public:
    ActuatorController() : Node("actuator_controller") {
        // --- Parameters ---
        this->declare_parameter("max_servo_angle", 180.0);
        this->declare_parameter("max_stepper_steps", 4096.0); // 28BYJ-48 Half-step full rotation
        
        max_servo_ = this->get_parameter("max_servo_angle").as_double();
        max_stepper_ = this->get_parameter("max_stepper_steps").as_double();

        // --- Hardware Setup ---
        h_ = lgGpiochipOpen(4);
        
        // Servo Pin (MG-995)
        servo_pin_ = 12; 
        lgGpioClaimOutput(h_, 0, servo_pin_, 0);

        // Stepper Pins (28BYJ-48 / ULN2003)
        stepper_pins_ = {17, 18, 27, 22}; // IN1, IN2, IN3, IN4
        for (int pin : stepper_pins_) {
            lgGpioClaimOutput(h_, 0, pin, 0);
        }

        // --- ROS2 Subscription ---
        // Expected data: [stepper_target_steps, servo_target_degrees]
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/actuator_cmd", 10, std::bind(&ActuatorController::callback, this, std::placeholders::_1));

        // Timer for stepper movement (non-blocking)
        stepper_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2), std::bind(&ActuatorController::move_stepper, this));

        RCLCPP_INFO(this->get_logger(), "Actuators initialized. Waiting for commands...");
    }

    ~ActuatorController() {
        lgGpiochipClose(h_);
    }

private:
    int h_;
    int servo_pin_;
    std::vector<int> stepper_pins_;
    double max_servo_, max_stepper_;
    
    int current_step_ = 0;
    int target_step_ = 0;
    int step_index_ = 0;

    // Half-step sequence for 28BYJ-48
    const int sequence[8][4] = {
        {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
        {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
    };

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr stepper_timer_;

    void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;

        // 1. Handle Stepper (Input is target step count)
        target_step_ = std::clamp((int)msg->data[0], 0, (int)max_stepper_);

        // 2. Handle Servo (Input is degrees 0-180)
        double angle = std::clamp(msg->data[1], 0.0, max_servo_);
        set_servo_angle(angle);
    }

    void set_servo_angle(double angle) {
        // MG-995: 50Hz (20ms period). 
        // 0.5ms (2.5% duty) -> 0 deg | 2.5ms (12.5% duty) -> 180 deg
        float duty_cycle = 2.5f + (angle / 180.0f) * 10.0f; 
        lgTxPwm(h_, servo_pin_, 50.0f, duty_cycle, 0, 0);
    }

    void move_stepper() {
        if (current_step_ == target_step_) return;

        // Determine direction
        if (current_step_ < target_step_) {
            step_index_ = (step_index_ + 1) % 8;
            current_step_++;
        } else {
            step_index_ = (step_index_ - 1 + 8) % 8;
            current_step_--;
        }

        // Apply sequence to pins
        for (int i = 0; i < 4; ++i) {
            lgGpioWrite(h_, stepper_pins_[i], sequence[step_index_][i]);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorController>());
    rclcpp::shutdown();
    return 0;
}
