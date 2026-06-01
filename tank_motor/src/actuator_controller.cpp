/* 
NOTE: File is ~270 lines due to hardware driver complexity; splitting would create
      artificial awkwardness across the stepper/servo subsystems.

Platform: Raspberry Pi 5 · Ubuntu 24 · ROS2 Jazzy
Dependencies: lgpio (stepper), kernel PWM sysfs (servo), rclcpp, std_msgs

--- Kernel PWM setup (run once, survives reboot with a udev rule) ----------
  sudo sh -c 'echo "dtoverlay=pwm,pin=12,func=4" >> /boot/firmware/config.txt'
  sudo reboot
  # After reboot, export PWM channel so the node can access it without root:
  echo 0 | sudo tee /sys/class/pwm/pwmchip2/export
  sudo chmod a+rw /sys/class/pwm/pwmchip2/pwm0/period \
                  /sys/class/pwm/pwmchip2/pwm0/duty_cycle \
                  /sys/class/pwm/pwmchip2/pwm0/enable
  # OR add a udev rule so permissions persist across reboots:
  # echo 'SUBSYSTEM=="pwm*", PROGRAM="/bin/sh -c 'chown -R root:gpio /sys/class/pwm && chmod -R 770 /sys/class/pwm'"' \
  #   | sudo tee /etc/udev/rules.d/99-pwm.rules

--- Install dependencies ---------------------------------------------------
  sudo apt update && sudo apt install -y liblgpio-dev
  (ROS2 Jazzy assumed already installed)

--- Build ------------------------------------------------------------------
  CMakeLists.txt must link: lgpio rclcpp std_msgs
  colcon build --packages-select <your_package>
*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <lgpio.h>
#include <sys/stat.h>  // <-- FIXES THE COMPILATION ERROR
#include <sys/types.h> // Good practice when working with system calls
#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Hardware constants
// ---------------------------------------------------------------------------

// GPIO chip: 4 on RPi 5, 0 on RPi 4. Exposed as a ROS2 parameter below.
static constexpr int DEFAULT_GPIOCHIP        = 4;

// Servo (MG995) — hardware PWM via kernel sysfs
// GPIO 12 = PWM0 on RPi 5 RP1 chip → pwmchip2/pwm0
// Period: 20 000 000 ns (50 Hz). Pulse: 1 000 000 ns (1 ms) = 0°, 2 000 000 ns (2 ms) = 180°
static constexpr char   PWM_CHIP_PATH[]        = "/sys/class/pwm/pwmchip2";
static constexpr char   PWM_CHANNEL[]          = "pwm0";
static constexpr long   SERVO_PERIOD_NS        = 20'000'000L;  // 50 Hz
static constexpr long   SERVO_PULSE_MIN_NS     = 1'000'000L;   // 1 ms → 0 °
static constexpr long   SERVO_PULSE_MAX_NS     = 2'000'000L;   // 2 ms → 180 °
static constexpr double SERVO_ANGLE_MIN        = 0.0;
static constexpr double SERVO_ANGLE_MAX        = 180.0;

// Stepper (28BYJ-48 via ULN2003A) — half-step, 4096 steps/rev
static constexpr int STEPPER_STEPS_PER_REV   = 4096;
static constexpr int STEPPER_SEQ_LENGTH      = 8;
static constexpr int STEPPER_PIN_COUNT       = 4;
static constexpr int STEPPER_TIMER_MS        = 6;      // step interval — do not go below 1 ms

// IN1, IN2, IN3, IN4 on the ULN2003A board
static constexpr int STEPPER_PINS[STEPPER_PIN_COUNT]  = {16, 26, 20, 21};

// Half-step sequence for 28BYJ-48
static constexpr int HALF_STEP_SEQ[STEPPER_SEQ_LENGTH][STEPPER_PIN_COUNT] = {
    {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
    {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

// ROS2 topic
static constexpr char ACTUATOR_CMD_TOPIC[]   = "/actuator_cmd";
static constexpr int  SUBSCRIPTION_QUEUE     = 10;

// ---------------------------------------------------------------------------
// ActuatorController
// ---------------------------------------------------------------------------

class ActuatorController : public rclcpp::Node {
public:
    ActuatorController() : Node("actuator_controller") {
        declare_parameters();
        open_gpio_chip();
        claim_stepper_pins();   // FIX 1: odkomentowane — bez tego lgGpioWrite() jest ignorowane
        init_hardware_pwm();
        setup_subscriptions();
        create_stepper_timer();
        RCLCPP_INFO(get_logger(), "Actuators initialised on gpiochip%d. Waiting for commands…",
                    chip_id_);
    }

    ~ActuatorController() {
        deenergise_stepper();
        stop_servo_pwm();
        lgGpiochipClose(gpio_handle_);
    }

private:
    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------

    int gpio_handle_  = -1;
    int chip_id_      = DEFAULT_GPIOCHIP;

    std::string pwm_channel_path_;   // e.g. /sys/class/pwm/pwmchip2/pwm0

    double max_servo_angle_   = SERVO_ANGLE_MAX;
    int    max_stepper_steps_ = STEPPER_STEPS_PER_REV;

    int current_step_ = 0;
    int step_index_   = 0;
    std::atomic<int> target_step_{0};   // written by ROS2 callback, read by timer

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr stepper_timer_;

    // -----------------------------------------------------------------------
    // Initialisation helpers
    // -----------------------------------------------------------------------

    void declare_parameters() {
        declare_parameter("gpiochip",          DEFAULT_GPIOCHIP);
        declare_parameter("max_servo_angle",   SERVO_ANGLE_MAX);
        declare_parameter("max_stepper_steps", static_cast<double>(STEPPER_STEPS_PER_REV));

        chip_id_           = get_parameter("gpiochip").as_int();
        max_servo_angle_   = get_parameter("max_servo_angle").as_double();
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

    // FIX 1: odkomentowana definicja — lgGpioWrite() wymaga wcześniejszego claim
    void claim_stepper_pins() {
    for (int pin : STEPPER_PINS) {
        // Trzeci parametr w lgGpioClaimOutput określa flagi, 
        // a czwarty (0) to domyślny stan (LOW).
        check_lgpio(
            lgGpioClaimOutput(gpio_handle_, 0, pin, 0), 
            "claim stepper pin GPIO " + std::to_string(pin));
    }
    // Dodatkowo od razu wymuś stan niski na start
    deenergise_stepper(); 
    RCLCPP_INFO(get_logger(), "Stepper pins claimed and set to LOW.");
}

    // Initialise the kernel hardware PWM channel via sysfs.
    // Requires the PWM overlay to be loaded (see setup instructions at top of file).
    //
    // FIX 2: Kolejność operacji sysfs jest krytyczna dla kernela:
    //   1. Wyłącz PWM (enable=0) jeśli już był aktywny
    //   2. Ustaw period
    //   3. Ustaw duty_cycle (musi być <= period, a przy zmianie period kernel może
    //      odrzucić duty_cycle > nowy period — dlatego najpierw period, potem duty)
    //   4. Włącz PWM (enable=1)
    // Każdy write_sysfs() robi .flush() by wymusić zapis do kernela przed kolejnym krokiem.
   void init_hardware_pwm() {
    pwm_channel_path_ = std::string(PWM_CHIP_PATH) + "/" + PWM_CHANNEL;
    
    // Check if the pwm channel directory already exists using standard POSIX stat
    struct stat st{};
    if (::stat(pwm_channel_path_.c_str(), &st) != 0) {
        RCLCPP_INFO(get_logger(), "Exporting PWM channel %s...", PWM_CHANNEL);
        write_sysfs(std::string(PWM_CHIP_PATH) + "/export", "0");
        // Give the kernel a moment to create the sysfs directory structure
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    // FIX 2a: Disable before reconfiguration — changing period on active channel
    // is rejected by the kernel on certain RP1 PWM driver versions
    write_sysfs(pwm_channel_path_ + "/enable", "0");

    // FIX 2b: Set duty_cycle=0 before changing period to avoid kernel race condition
    write_sysfs(pwm_channel_path_ + "/duty_cycle", "0");

    // FIX 2c: Now safely set target period
    write_sysfs(pwm_channel_path_ + "/period", std::to_string(SERVO_PERIOD_NS));

    // FIX 2d: Set start position (0°) and enable
    write_sysfs(pwm_channel_path_ + "/duty_cycle", std::to_string(SERVO_PULSE_MIN_NS));
    write_sysfs(pwm_channel_path_ + "/enable", "1");

    RCLCPP_INFO(get_logger(), "Hardware PWM initialised at %s (period=%ldns, duty=%ldns)",
                pwm_channel_path_.c_str(), SERVO_PERIOD_NS, SERVO_PULSE_MIN_NS);
}

    void setup_subscriptions() {
        subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            ACTUATOR_CMD_TOPIC, SUBSCRIPTION_QUEUE,
            std::bind(&ActuatorController::on_actuator_cmd, this, std::placeholders::_1));
    }

    void create_stepper_timer() {
        stepper_timer_ = create_wall_timer(
            std::chrono::milliseconds(STEPPER_TIMER_MS),
            std::bind(&ActuatorController::advance_stepper, this));
    }

    // -----------------------------------------------------------------------
    // ROS2 callback
    // -----------------------------------------------------------------------

    void on_actuator_cmd(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(get_logger(), "Expected 2 floats [stepper_steps, servo_degrees], got %zu",
                        msg->data.size());
            return;
        }
        target_step_.store(
            std::clamp(static_cast<int>(msg->data[0]), 0, max_stepper_steps_));

        double angle = std::clamp(msg->data[1], 0.0, max_servo_angle_);
        set_servo_angle(angle);
    }

    // -----------------------------------------------------------------------
    // Servo — kernel hardware PWM (jitter-free, no lgd dependency)
    // -----------------------------------------------------------------------

    void set_servo_angle(double angle) {
        // Map angle → pulse width in nanoseconds
        // 0° = SERVO_PULSE_MIN_NS (1 ms), 180° = SERVO_PULSE_MAX_NS (2 ms)
        long pulse_ns = SERVO_PULSE_MIN_NS
                      + static_cast<long>((angle / max_servo_angle_)
                        * static_cast<double>(SERVO_PULSE_MAX_NS - SERVO_PULSE_MIN_NS));

        // FIX 3: clamp dla bezpieczeństwa — duty_cycle > period zabija PWM driver
        pulse_ns = std::clamp(pulse_ns, SERVO_PULSE_MIN_NS, SERVO_PULSE_MAX_NS);

        write_sysfs(pwm_channel_path_ + "/duty_cycle", std::to_string(pulse_ns));
        RCLCPP_DEBUG(get_logger(), "Servo: %.1f° → %ld ns", angle, pulse_ns);
    }

    void stop_servo_pwm() {
        write_sysfs(pwm_channel_path_ + "/duty_cycle", std::to_string(SERVO_PULSE_MIN_NS));
        write_sysfs(pwm_channel_path_ + "/enable", "0");
    }

    // Write a string value to a sysfs file; throws on failure.
    // FIX 4: Dodano explicit flush() — bez tego dane mogą zostać w buforze
    // i kernel nie zobaczy wartości przed kolejną operacją (np. enable przed duty_cycle)
    void write_sysfs(const std::string& path, const std::string& value) {
        std::ofstream file(path);
        if (!file.is_open()) {
            RCLCPP_FATAL(get_logger(), "Cannot open sysfs path: %s", path.c_str());
            throw std::runtime_error("sysfs write failed: " + path);
        }
        file << value;
        file.flush();   // FIX 4: wymuszony flush przed zamknięciem
        if (file.fail()) {
            RCLCPP_FATAL(get_logger(), "Write failed on sysfs path: %s (value: %s)",
                         path.c_str(), value.c_str());
            throw std::runtime_error("sysfs write failed: " + path);
        }
    }

    // -----------------------------------------------------------------------
    // Stepper
    // -----------------------------------------------------------------------

    void advance_stepper() {
        int target = target_step_.load();

        if (current_step_ == target) {
            deenergise_stepper();
            return;
        }

        if (current_step_ < target) {
            step_index_ = (step_index_ + 1) % STEPPER_SEQ_LENGTH;
            ++current_step_;
        } else {
            step_index_ = (step_index_ - 1 + STEPPER_SEQ_LENGTH) % STEPPER_SEQ_LENGTH;
            --current_step_;
        }

        apply_step_sequence();
    }

    // FIX 5: Dodano sprawdzanie błędów lgGpioWrite — bez tego ciche faile
    // (np. gdy pin nie jest claimed) są niewidoczne
    void apply_step_sequence() {
        for (int i = 0; i < STEPPER_PIN_COUNT; ++i) {
            check_lgpio(
                lgGpioWrite(gpio_handle_, STEPPER_PINS[i], HALF_STEP_SEQ[step_index_][i]),
                "write stepper pin GPIO " + std::to_string(STEPPER_PINS[i]));
        }
    }

    void deenergise_stepper() {
        for (int pin : STEPPER_PINS) {
            lgGpioWrite(gpio_handle_, pin, 0);
        }
    }

    // -----------------------------------------------------------------------
    // Utility
    // -----------------------------------------------------------------------

    void check_lgpio(int return_code, const std::string& context) {
        if (return_code < 0) {
            RCLCPP_FATAL(get_logger(), "%s failed: %s",
                         context.c_str(), lguErrorText(return_code));
            throw std::runtime_error(context + " failed");
        }
    }
};

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorController>());
    rclcpp::shutdown();
    return 0;
}