/* 
Platform: Raspberry Pi 5 · Ubuntu 24 · ROS2 Jazzy
Chwytak: stepper 28BYJ-48 (szczęki, rack-pinion) + MG995 (ramię góra/dół)
*/

#include <rclcpp/rclcpp.hpp>
#include <dualtech_msgs/msg/actuator_cmd.hpp>
#include <lgpio.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

static constexpr int DEFAULT_GPIOCHIP        = 4;

static constexpr int    SERVO_GPIO_PIN         = 12;
static constexpr int    SERVO_PULSE_MIN_US     = 1'000;
static constexpr int    SERVO_PULSE_MAX_US     = 2'000;
static constexpr int    SERVO_FREQUENCY_HZ     = 50;

static constexpr int STEPPER_STEPS_PER_REV   = 4096;
static constexpr int STEPPER_MAX_STEPS       = STEPPER_STEPS_PER_REV;
static constexpr int STEPPER_SEQ_LENGTH      = 8;
static constexpr int STEPPER_PIN_COUNT       = 4;
static constexpr int STEPPER_TIMER_US        = 2'000;

static constexpr int STEPPER_PINS[STEPPER_PIN_COUNT]  = {16, 26, 20, 21};

static constexpr int HALF_STEP_SEQ[STEPPER_SEQ_LENGTH][STEPPER_PIN_COUNT] = {
    {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
    {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

static constexpr char ACTUATOR_CMD_TOPIC[]   = "/actuator_cmd";
static constexpr int  SUBSCRIPTION_QUEUE     = 10;

class ActuatorController : public rclcpp::Node {
public:
    ActuatorController() : Node("actuator_controller") {
        declare_parameters();
        open_gpio_chip();
        claim_stepper_pins();
        claim_servo_pin();
        setup_subscriptions();
        create_stepper_timer();
        RCLCPP_INFO(get_logger(),
                    "Chwytak gotowy (stepper=szczęki, MG995=ramię) na gpiochip%d",
                    chip_id_);
    }

    ~ActuatorController() {
        deenergise_stepper();
        stop_servo_pwm();
        lgGpiochipClose(gpio_handle_);
    }

private:
    int gpio_handle_  = -1;
    int chip_id_      = DEFAULT_GPIOCHIP;

    double min_servo_angle_ = 30.0;
    double max_servo_angle_ = 120.0;
    double home_arm_angle_  = 75.0;
    bool   invert_servo_    = true;
    bool   reverse_stepper_ = false;
    int    max_stepper_steps_ = STEPPER_MAX_STEPS;

    int current_step_ = 0;
    int step_index_   = 0;
    std::atomic<int> target_step_{0};
    double last_arm_angle_ = -1.0;

    rclcpp::Subscription<dualtech_msgs::msg::ActuatorCmd>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr stepper_timer_;

    void declare_parameters() {
        declare_parameter("gpiochip",          DEFAULT_GPIOCHIP);
        declare_parameter("min_servo_angle",   min_servo_angle_);
        declare_parameter("max_servo_angle",   max_servo_angle_);
        declare_parameter("home_arm_angle",    home_arm_angle_);
        declare_parameter("invert_servo",      invert_servo_);
        declare_parameter("reverse_stepper",   reverse_stepper_);
        declare_parameter("max_stepper_steps", static_cast<double>(STEPPER_MAX_STEPS));

        chip_id_           = get_parameter("gpiochip").as_int();
        min_servo_angle_   = get_parameter("min_servo_angle").as_double();
        max_servo_angle_   = get_parameter("max_servo_angle").as_double();
        home_arm_angle_    = get_parameter("home_arm_angle").as_double();
        invert_servo_      = get_parameter("invert_servo").as_bool();
        reverse_stepper_   = get_parameter("reverse_stepper").as_bool();
        max_stepper_steps_ = static_cast<int>(get_parameter("max_stepper_steps").as_double());
    }

    void open_gpio_chip() {
        gpio_handle_ = lgGpiochipOpen(chip_id_);
        if (gpio_handle_ < 0) {
            RCLCPP_FATAL(get_logger(), "Cannot open gpiochip%d: %s",
                         chip_id_, lguErrorText(gpio_handle_));
            throw std::runtime_error("lgGpiochipOpen failed");
        }
    }

    void claim_stepper_pins() {
        for (int pin : STEPPER_PINS) {
            check_lgpio(
                lgGpioClaimOutput(gpio_handle_, 0, pin, 0),
                "claim stepper pin GPIO " + std::to_string(pin));
        }
        deenergise_stepper();
        RCLCPP_INFO(get_logger(), "Stepper (szczęki) — piny zainicjalizowane.");
    }

    void claim_servo_pin() {
        check_lgpio(
            lgGpioClaimOutput(gpio_handle_, 0, SERVO_GPIO_PIN, 0),
            "claim servo pin GPIO " + std::to_string(SERVO_GPIO_PIN));
        set_servo_angle(home_arm_angle_);
        RCLCPP_INFO(get_logger(),
                    "MG995 (ramię) — GPIO %d, zakres %.0f°–%.0f°, invert=%s",
                    SERVO_GPIO_PIN, min_servo_angle_, max_servo_angle_,
                    invert_servo_ ? "tak" : "nie");
    }

    void setup_subscriptions() {
        subscription_ = create_subscription<dualtech_msgs::msg::ActuatorCmd>(
            ACTUATOR_CMD_TOPIC, SUBSCRIPTION_QUEUE,
            std::bind(&ActuatorController::on_actuator_cmd, this, std::placeholders::_1));
    }

    void create_stepper_timer() {
        stepper_timer_ = create_wall_timer(
            std::chrono::microseconds(STEPPER_TIMER_US),
            std::bind(&ActuatorController::advance_stepper, this));
    }

    void on_actuator_cmd(const dualtech_msgs::msg::ActuatorCmd::SharedPtr msg) {
        const int new_target = std::clamp(
            static_cast<int>(msg->jaw_position), 0, max_stepper_steps_);
        const int old_target = target_step_.load();

        if (new_target != old_target) {
            target_step_.store(new_target);
            RCLCPP_INFO(get_logger(),
                        "Stepper → cel: %d kroków (obecnie: %d)", new_target, current_step_);
        }

        const double angle = std::clamp(msg->arm_angle_deg, min_servo_angle_, max_servo_angle_);
        if (std::abs(angle - last_arm_angle_) > 0.1) {
            set_servo_angle(angle);
            last_arm_angle_ = angle;
        }
    }

    void set_servo_angle(double angle) {
        const double span = max_servo_angle_ - min_servo_angle_;
        double normalized = (span > 0.0)
            ? (angle - min_servo_angle_) / span
            : 0.0;

        if (invert_servo_) {
            normalized = 1.0 - normalized;
        }

        int pulse_us = SERVO_PULSE_MIN_US
            + static_cast<int>(normalized
                * static_cast<double>(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US));
        pulse_us = std::clamp(pulse_us, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);

        check_lgpio(
            lgTxServo(gpio_handle_, SERVO_GPIO_PIN, pulse_us,
                      SERVO_FREQUENCY_HZ, 0, 0),
            "servo pulse");
        RCLCPP_DEBUG(get_logger(), "MG995: %.1f° → %d µs", angle, pulse_us);
    }

    void stop_servo_pwm() {
        lgTxServo(gpio_handle_, SERVO_GPIO_PIN, 0, SERVO_FREQUENCY_HZ, 0, 0);
    }

    void advance_stepper() {
        const int target = target_step_.load();

        if (current_step_ == target) {
            deenergise_stepper();
            return;
        }

        const bool forward = current_step_ < target;
        if (forward == !reverse_stepper_) {
            step_index_ = (step_index_ + 1) % STEPPER_SEQ_LENGTH;
            ++current_step_;
        } else {
            step_index_ = (step_index_ - 1 + STEPPER_SEQ_LENGTH) % STEPPER_SEQ_LENGTH;
            --current_step_;
        }

        apply_step_sequence();
    }

    void apply_step_sequence() {
        for (int i = 0; i < STEPPER_PIN_COUNT; ++i) {
            const int level = HALF_STEP_SEQ[step_index_][i];
            const int rc = lgGpioWrite(gpio_handle_, STEPPER_PINS[i], level);
            if (rc < 0) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Stepper GPIO %d write failed: %s",
                    STEPPER_PINS[i], lguErrorText(rc));
            }
        }
    }

    void deenergise_stepper() {
        for (int pin : STEPPER_PINS) {
            lgGpioWrite(gpio_handle_, pin, 0);
        }
    }

    void check_lgpio(int return_code, const std::string& context) {
        if (return_code < 0) {
            RCLCPP_FATAL(get_logger(), "%s failed: %s",
                         context.c_str(), lguErrorText(return_code));
            throw std::runtime_error(context + " failed");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorController>());
    rclcpp::shutdown();
    return 0;
}
