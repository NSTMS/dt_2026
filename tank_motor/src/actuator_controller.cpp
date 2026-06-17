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

static constexpr int STEPPER_HALF_STEPS_PER_REV = 4096;
static constexpr int STEPPER_FULL_STEPS_PER_REV = 2048;
static constexpr int STEPPER_HALF_SEQ_LENGTH    = 8;
static constexpr int STEPPER_FULL_SEQ_LENGTH    = 4;
static constexpr int STEPPER_PIN_COUNT          = 4;
static constexpr int DEFAULT_STEP_DELAY_US      = 900;
static constexpr int MAX_STEPS_PER_TICK         = 4;

static constexpr int STEPPER_PINS[STEPPER_PIN_COUNT]  = {16, 26, 20, 21};

static constexpr int HALF_STEP_SEQ[STEPPER_HALF_SEQ_LENGTH][STEPPER_PIN_COUNT] = {
    {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
    {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

static constexpr int FULL_STEP_SEQ[STEPPER_FULL_SEQ_LENGTH][STEPPER_PIN_COUNT] = {
    {1, 1, 0, 0}, {0, 1, 1, 0}, {0, 0, 1, 1}, {1, 0, 0, 1}
};

static constexpr char ACTUATOR_CMD_TOPIC[]   = "/actuator_cmd";
static constexpr int  SUBSCRIPTION_QUEUE     = 10;

class ActuatorController : public rclcpp::Node {
public:
    ActuatorController() : Node("actuator_node") {
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

    double home_arm_angle_  = 90.0;
    bool   invert_servo_    = true;
    bool   reverse_stepper_ = false;
    bool   use_half_step_   = true;
    int    step_delay_us_   = DEFAULT_STEP_DELAY_US;
    int    seq_length_      = STEPPER_HALF_SEQ_LENGTH;

    int current_step_ = 0;
    int step_index_   = 0;
    std::atomic<int> target_step_{0};
    double last_arm_angle_ = -1.0;

    rclcpp::Subscription<dualtech_msgs::msg::ActuatorCmd>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr stepper_timer_;

    void declare_parameters() {
        declare_parameter("gpiochip",          DEFAULT_GPIOCHIP);
        declare_parameter("home_arm_angle",    home_arm_angle_);
        declare_parameter("invert_servo",      invert_servo_);
        declare_parameter("reverse_stepper",   reverse_stepper_);
        declare_parameter("use_half_step",     use_half_step_);
        declare_parameter("step_delay_us",     static_cast<double>(DEFAULT_STEP_DELAY_US));

        chip_id_           = get_parameter("gpiochip").as_int();
        home_arm_angle_    = get_parameter("home_arm_angle").as_double();
        invert_servo_      = get_parameter("invert_servo").as_bool();
        reverse_stepper_   = get_parameter("reverse_stepper").as_bool();
        use_half_step_     = get_parameter("use_half_step").as_bool();
        step_delay_us_     = std::max(400, static_cast<int>(get_parameter("step_delay_us").as_double()));

        seq_length_    = use_half_step_ ? STEPPER_HALF_SEQ_LENGTH : STEPPER_FULL_SEQ_LENGTH;
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
        RCLCPP_INFO(get_logger(),
                    "Stepper (szczęki) — %s, opóźnienie %d µs, bez limitu kroków",
                    use_half_step_ ? "half-step" : "full-step",
                    step_delay_us_);
    }

    void claim_servo_pin() {
        check_lgpio(
            lgGpioClaimOutput(gpio_handle_, 0, SERVO_GPIO_PIN, 0),
            "claim servo pin GPIO " + std::to_string(SERVO_GPIO_PIN));
        set_servo_angle(home_arm_angle_);
        last_arm_angle_ = home_arm_angle_;
        RCLCPP_INFO(get_logger(),
                    "MG995 (ramię) — GPIO %d, bez limitów kąta, invert=%s",
                    SERVO_GPIO_PIN,
                    invert_servo_ ? "tak" : "nie");
    }

    void setup_subscriptions() {
        subscription_ = create_subscription<dualtech_msgs::msg::ActuatorCmd>(
            ACTUATOR_CMD_TOPIC, SUBSCRIPTION_QUEUE,
            std::bind(&ActuatorController::on_actuator_cmd, this, std::placeholders::_1));
    }

    void create_stepper_timer() {
        stepper_timer_ = create_wall_timer(
            std::chrono::microseconds(step_delay_us_),
            std::bind(&ActuatorController::advance_stepper, this));
    }

    void on_actuator_cmd(const dualtech_msgs::msg::ActuatorCmd::SharedPtr msg) {
        const int new_target = static_cast<int>(msg->jaw_position);
        const int old_target = target_step_.load();

        if (new_target != old_target) {
            target_step_.store(new_target);
            RCLCPP_INFO(get_logger(),
                        "Stepper → cel: %d kroków (obecnie: %d)", new_target, current_step_);
        }

        const double angle = msg->arm_angle_deg;
        if (std::abs(angle - last_arm_angle_) > 0.1) {
            set_servo_angle(angle);
            last_arm_angle_ = angle;
        }
    }

    void set_servo_angle(double angle) {
        double normalized = angle / 180.0;

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

        const int remaining = std::abs(target - current_step_);
        const int burst = std::min(
            MAX_STEPS_PER_TICK,
            std::max(1, remaining / 128));

        for (int n = 0; n < burst && current_step_ != target; ++n) {
            step_once_toward(target);
        }

        apply_step_sequence();
    }

    void step_once_toward(int target) {
        const bool forward = current_step_ < target;
        if (forward == !reverse_stepper_) {
            step_index_ = (step_index_ + 1) % seq_length_;
            ++current_step_;
        } else {
            step_index_ = (step_index_ - 1 + seq_length_) % seq_length_;
            --current_step_;
        }
    }

    void apply_step_sequence() {
        for (int i = 0; i < STEPPER_PIN_COUNT; ++i) {
            const int level = use_half_step_
                ? HALF_STEP_SEQ[step_index_][i]
                : FULL_STEP_SEQ[step_index_][i];
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
