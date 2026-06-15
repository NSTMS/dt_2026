#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <dualtech_msgs/msg/actuator_cmd.hpp>

#include <algorithm>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

static struct termios g_orig_termios;
static int g_tty_fd = -1;

static void restoreTerminal()
{
    if (g_tty_fd >= 0) {
        tcsetattr(g_tty_fd, TCSANOW, &g_orig_termios);
    }
}

static bool setRawTerminal()
{
    g_tty_fd = open("/dev/tty", O_RDWR | O_NOCTTY);
    if (g_tty_fd < 0) {
        g_tty_fd = STDIN_FILENO;
    }

    if (!isatty(g_tty_fd)) {
        return false;
    }

    tcgetattr(g_tty_fd, &g_orig_termios);
    atexit(restoreTerminal);

    struct termios raw = g_orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(g_tty_fd, TCSANOW, &raw);

    int flags = fcntl(g_tty_fd, F_GETFL, 0);
    fcntl(g_tty_fd, F_SETFL, flags | O_NONBLOCK);
    return true;
}

class KeyboardController : public rclcpp::Node
{
public:
    KeyboardController() : Node("keyboard_controller")
    {
        declare_parameter("max_stepper_steps", 4096.0);
        declare_parameter("min_servo_angle", 30.0);
        declare_parameter("max_servo_angle", 120.0);
        declare_parameter("home_arm_angle", 75.0);
        declare_parameter("stepper_increment", 128.0);
        declare_parameter("servo_increment", 5.0);
        declare_parameter("cmd_vel_rate_hz", 10.0);

        max_stepper_ = get_parameter("max_stepper_steps").as_double();
        min_servo_ = get_parameter("min_servo_angle").as_double();
        max_servo_ = get_parameter("max_servo_angle").as_double();
        home_arm_ = get_parameter("home_arm_angle").as_double();
        stepper_inc_ = get_parameter("stepper_increment").as_double();
        servo_inc_ = get_parameter("servo_increment").as_double();

        const double rate_hz = get_parameter("cmd_vel_rate_hz").as_double();
        const auto period_ms = static_cast<int>(1000.0 / std::max(rate_hz, 1.0));

        jaw_pos_ = 0.0;
        arm_angle_ = home_arm_;

        motor_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        act_pub_ = create_publisher<dualtech_msgs::msg::ActuatorCmd>("/actuator_cmd", 10);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&KeyboardController::timerCb, this));

        tty_ok_ = setRawTerminal();
        if (tty_ok_) {
            printHelp();
            publishActuator(jaw_pos_, arm_angle_);
        } else {
            RCLCPP_ERROR(get_logger(),
                "Brak TTY — uruchom w interaktywnym terminalu:\n"
                "  ros2 run tank_motor keyboard_controller");
        }
    }

    ~KeyboardController()
    {
        motor_pub_->publish(geometry_msgs::msg::Twist{});
        publishActuator(0.0, home_arm_);
        restoreTerminal();
    }

private:
    rclcpp::Publisher<dualtech_msgs::msg::ActuatorCmd>::SharedPtr act_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    static constexpr float SPEED_STEP = 0.05f;
    static constexpr float SPEED_MIN = 0.05f;
    static constexpr float SPEED_MAX = 0.6f;

    double max_stepper_ = 4096.0;
    double min_servo_ = 30.0;
    double max_servo_ = 120.0;
    double home_arm_ = 75.0;
    double stepper_inc_ = 128.0;
    double servo_inc_ = 5.0;

    double jaw_pos_ = 0.0;
    double arm_angle_ = 75.0;

    float linear_speed_ = 0.35f;
    float angular_speed_ = 0.35f;
    geometry_msgs::msg::Twist active_twist_{};
    geometry_msgs::msg::Twist last_published_twist_{};
    bool tty_ok_ = false;

    void printHelp()
    {
        RCLCPP_INFO(get_logger(),
                    "Sterowanie (/dev/tty) — czołg + chwytak.\n"
                    "  W / S           – jazda przód / tył\n"
                    "  A / D           – skręt lewo / prawo\n"
                    "  Spacja          – stop + pozycja domowa chwytaka\n"
                    "  Strzałka Góra/Dół – zmiana prędkości\n"
                    "  H / K           – szczęki: zamknij / otwórz (stepper)\n"
                    "  U / J           – ramię: w dół / w górę (MG995)\n"
                    "  Q               – wyjście\n"
                    "  Prędkość: linear=%.2f  angular=%.2f",
                    linear_speed_, angular_speed_);
    }

    enum class Key
    {
        None, W, A, S, D, U, J, H, K, Space, ArrowUp, ArrowDown, Quit
    };

    Key readKey()
    {
        if (!tty_ok_) {
            return Key::None;
        }

        char buf[8]{};
        const ssize_t n = read(g_tty_fd, buf, sizeof(buf));
        if (n <= 0) {
            return Key::None;
        }

        if (n >= 3 && buf[0] == '\x1b' && buf[1] == '[')
        {
            switch (buf[2])
            {
            case 'A': return Key::ArrowUp;
            case 'B': return Key::ArrowDown;
            default:  return Key::None;
            }
        }

        switch (buf[n - 1])
        {
        case 'w': case 'W': return Key::W;
        case 's': case 'S': return Key::S;
        case 'a': case 'A': return Key::A;
        case 'd': case 'D': return Key::D;
        case ' ':           return Key::Space;
        case 'u': case 'U': return Key::U;
        case 'j': case 'J': return Key::J;
        case 'h': case 'H': return Key::H;
        case 'k': case 'K': return Key::K;
        case 'q': case 'Q': return Key::Quit;
        default:            return Key::None;
        }
    }

    void publishActuator(double jaw, double arm)
    {
        dualtech_msgs::msg::ActuatorCmd msg;
        msg.jaw_position = jaw;
        msg.arm_angle_deg = arm;
        act_pub_->publish(msg);
    }

    static bool twistsEqual(const geometry_msgs::msg::Twist & a,
                            const geometry_msgs::msg::Twist & b)
    {
        return std::abs(a.linear.x - b.linear.x) < 1e-4f
            && std::abs(a.angular.z - b.angular.z) < 1e-4f;
    }

    void publishTwistIfChanged()
    {
        if (twistsEqual(active_twist_, last_published_twist_)) {
            return;
        }
        motor_pub_->publish(active_twist_);
        last_published_twist_ = active_twist_;
    }

    void processKey(Key key)
    {
        if (key == Key::ArrowUp || key == Key::ArrowDown)
        {
            const float delta = (key == Key::ArrowUp) ? SPEED_STEP : -SPEED_STEP;
            linear_speed_ = std::clamp(linear_speed_ + delta, SPEED_MIN, SPEED_MAX);
            angular_speed_ = std::clamp(angular_speed_ + delta, SPEED_MIN, SPEED_MAX);
            RCLCPP_INFO(get_logger(),
                        "Prędkość → linear=%.2f  angular=%.2f",
                        linear_speed_, angular_speed_);
            return;
        }

        bool act_changed = false;
        bool motor_changed = false;

        switch (key)
        {
        case Key::W:
            active_twist_.linear.x = linear_speed_;
            active_twist_.angular.z = 0.0f;
            motor_changed = true;
            break;
        case Key::S:
            active_twist_.linear.x = -linear_speed_;
            active_twist_.angular.z = 0.0f;
            motor_changed = true;
            break;
        case Key::A:
            active_twist_.linear.x = 0.0f;
            active_twist_.angular.z = angular_speed_;
            motor_changed = true;
            break;
        case Key::D:
            active_twist_.linear.x = 0.0f;
            active_twist_.angular.z = -angular_speed_;
            motor_changed = true;
            break;
        case Key::Space:
            active_twist_ = geometry_msgs::msg::Twist{};
            jaw_pos_ = 0.0;
            arm_angle_ = home_arm_;
            motor_changed = true;
            act_changed = true;
            break;
        case Key::H:
            jaw_pos_ = std::min(jaw_pos_ + stepper_inc_, max_stepper_);
            act_changed = true;
            break;
        case Key::K:
            jaw_pos_ = std::max(jaw_pos_ - stepper_inc_, 0.0);
            act_changed = true;
            break;
        case Key::U:
            arm_angle_ = std::max(arm_angle_ - servo_inc_, min_servo_);
            act_changed = true;
            break;
        case Key::J:
            arm_angle_ = std::min(arm_angle_ + servo_inc_, max_servo_);
            act_changed = true;
            break;
        case Key::Quit:
            RCLCPP_INFO(get_logger(), "Wyjście.");
            rclcpp::shutdown();
            return;
        default:
            break;
        }

        if (motor_changed) {
            publishTwistIfChanged();
            RCLCPP_INFO(get_logger(),
                        "Napęd → linear=%.2f  angular=%.2f",
                        active_twist_.linear.x, active_twist_.angular.z);
        }

        if (act_changed)
        {
            publishActuator(jaw_pos_, arm_angle_);
            RCLCPP_INFO(get_logger(), "Chwytak → szczęki: %.0f | ramię: %.1f°",
                        jaw_pos_, arm_angle_);
        }
    }

    void timerCb()
    {
        if (!tty_ok_) {
            return;
        }

        const Key key = readKey();
        if (key != Key::None) {
            processKey(key);
        }

        // Odśwież watchdog tylko gdy jest aktywny ruch (bez ponownego ustawiania PWM)
        if (!twistsEqual(active_twist_, geometry_msgs::msg::Twist{})) {
            motor_pub_->publish(active_twist_);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardController>());
    rclcpp::shutdown();
    return 0;
}
