#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// ── Terminal helpers ──────────────────────────────────────────────────────────

static struct termios g_orig_termios;

static void restoreTerminal()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
}

static void setRawTerminal()
{
    tcgetattr(STDIN_FILENO, &g_orig_termios);
    atexit(restoreTerminal);

    struct termios raw = g_orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

// ── Node ─────────────────────────────────────────────────────────────────────

class KeyboardController : public rclcpp::Node
{
public:
    KeyboardController() : Node("keyboard_controller")
    {
        motor_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        act_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/actuator_cmd", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&KeyboardController::timerCb, this));

        setRawTerminal();
        printHelp();
    }

    ~KeyboardController()
    {
        motor_pub_->publish(geometry_msgs::msg::Twist{});
        act_pub_->publish(std_msgs::msg::Float64MultiArray{});
        restoreTerminal();
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr act_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    static constexpr float SPEED_STEP = 0.05f;
    static constexpr float SPEED_MIN = 0.05f;
    static constexpr float SPEED_MAX = 1.00f;

    // Constants for steps
    static constexpr float STEPPER_INC = 128.0; // Steps per keypress
    static constexpr float SERVO_INC = 5.0;     // Degrees per keypress

    static constexpr double MAX_STEPPER = 4096.0;
    static constexpr double MAX_SERVO = 180.0;
    // Current states
    double stepper_pos_ = 0.0;
    double servo_pos_ = 90.0; // Start at center

    float linear_speed_ = 0.5f;
    float angular_speed_ = 0.5f;

    void printHelp()
    {
        RCLCPP_INFO(this->get_logger(),
                    "Keyboard controller started.\n"
                    "  W / S        – drive forward / backward\n"
                    "  A / D        – turn left / right\n"
                    "  Space        – full stop\n"
                    "  Arrow Up/Dn  – increase / decrease speed (linear & angular)\n"
                    "  H / K  – Stepper: Increase / Decrease steps\n"
                    "  U / J  – Servo:   Increase / Decrease degrees\n"
                    "  Q            – quit\n"
                    "  Speed: linear=%.2f  angular=%.2f",
                    linear_speed_, angular_speed_);
    }

    // Drain all pending bytes and return the last meaningful key.
    // Arrow keys arrive as the 3-byte sequence ESC [ A/B/C/D.
    // We read everything available and decide after.
    enum class Key
    {
        None,
        W,
        A,
        S,
        D,
        U,
        J,
        H,
        K,
        Space,
        ArrowUp,
        ArrowDown,
        Quit
    };

    Key readKey()
    {
        // Read up to 8 bytes to capture any escape sequence in one shot
        char buf[8]{};
        const ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
        if (n <= 0)
            return Key::None;

        // Arrow key: ESC [ A/B
        if (n >= 3 && buf[0] == '\x1b' && buf[1] == '[')
        {
            switch (buf[2])
            {
            case 'A':
                return Key::ArrowUp;
            case 'B':
                return Key::ArrowDown;
            default:
                return Key::None;
            }
        }

        // Plain single-byte key (use last byte if somehow multiple arrived)
        switch (buf[n - 1])
        {
        case 'w':
        case 'W':
            return Key::W;
        case 's':
        case 'S':
            return Key::S;
        case 'a':
        case 'A':
            return Key::A;
        case 'd':
        case 'D':
            return Key::D;
        case ' ':
            return Key::Space;
        case 'u':
        case 'U':
            return Key::U;
        case 'j':
        case 'J':
            return Key::J;
        case 'h':
        case 'H':
            return Key::H;
        case 'k':
        case 'K':
            return Key::K;
        case 'q':
        case 'Q':
            return Key::Quit;
        default:
            return Key::None;
        }
    }

    void timerCb()
    {
        const Key key = readKey();
        if (key == Key::None)
            return;

        if (key == Key::ArrowUp || key == Key::ArrowDown)
        {
            const float delta = (key == Key::ArrowUp) ? SPEED_STEP : -SPEED_STEP;
            linear_speed_ = std::clamp(linear_speed_ + delta, SPEED_MIN, SPEED_MAX);
            angular_speed_ = std::clamp(angular_speed_ + delta, SPEED_MIN, SPEED_MAX);

            RCLCPP_INFO(this->get_logger(),
                        "Speed changed → linear=%.2f  angular=%.2f",
                        linear_speed_, angular_speed_);
            return; // speed change doesn't publish a motion command
        }

        bool motor_cmd = false;
        bool act_cmd = false;
        geometry_msgs::msg::Twist twist{};

        switch (key)
        {
        case Key::W:
            twist.linear.x = linear_speed_;
            motor_cmd = true;
            break;
        case Key::S:
            twist.linear.x = -linear_speed_;
            motor_cmd = true;
            break;
        case Key::A:
            twist.angular.z = angular_speed_;
            motor_cmd = true;
            break;
        case Key::D:
            twist.angular.z = -angular_speed_;
            motor_cmd = true;
            break;
        case Key::Space: /* zero twist – stops motors */
            motor_cmd = true;

            break;
        default:
            break;
        }

        switch (key)
        {
        case Key::H:
            stepper_pos_ = std::min(stepper_pos_ + STEPPER_INC, MAX_STEPPER);
            act_cmd = true;
            break;
        case Key::K:
            stepper_pos_ = std::max(stepper_pos_ - STEPPER_INC, 0.0);
            act_cmd = true;
            break;
        case Key::U:
            servo_pos_ = std::min(servo_pos_ + SERVO_INC, MAX_SERVO);
            act_cmd = true;
            break;
        case Key::J:
            servo_pos_ = std::max(servo_pos_ - SERVO_INC, 0.0);
            act_cmd = true;
            break;
        case Key::Space:
            stepper_pos_ = 0.0;
            servo_pos_ = 90.0;
            act_cmd = true;
            break;
        case Key::Quit:
            RCLCPP_INFO(this->get_logger(), "Quit requested.");
            rclcpp::shutdown();
            return;
        default:
            break;
        }

        if (motor_cmd)
        {
            motor_pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(),
                        "linear.x=%.1f  angular.z=%.2f", twist.linear.x, twist.angular.z);
        }

        if (act_cmd)
        {
            auto msg = std_msgs::msg::Float64MultiArray();
            msg.data = {stepper_pos_, servo_pos_};
            act_pub_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Sent -> Stepper: %.0f | Servo: %.1f",
                        stepper_pos_, servo_pos_);
        }
    }
};

// ── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardController>());
    rclcpp::shutdown();
    return 0;
}
