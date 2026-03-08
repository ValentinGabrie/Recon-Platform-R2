/**
 * @file motor_controller.cpp
 * @brief Motor controller node — subscribes to /cmd_vel and drives
 *        differential drive motors via pigpio PWM on Pi 5 GPIO.
 *
 * Real-time C++17 node. Implements a hardware watchdog that stops motors
 * if no cmd_vel is received within the configured timeout.
 */

#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>

#include <pigpio.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @class MotorController
 * @brief ROS2 node that converts Twist messages into differential drive
 *        PWM signals via pigpio.
 */
class MotorController : public rclcpp::Node
{
public:
    MotorController()
    : Node("motor_controller"),
      motors_stopped_(true)
    {
        // Declare parameters — loaded from hardware.yaml
        this->declare_parameter<int>("left_pwm_pin", 12);
        this->declare_parameter<int>("left_dir_pin", 16);
        this->declare_parameter<int>("right_pwm_pin", 13);
        this->declare_parameter<int>("right_dir_pin", 20);
        this->declare_parameter<int>("pwm_frequency_hz", 1000);
        this->declare_parameter<double>("wheel_separation_m", 0.20);
        this->declare_parameter<double>("wheel_radius_m", 0.033);
        this->declare_parameter<int>("watchdog_timeout_ms", 500);

        // Retrieve parameters
        left_pwm_pin_ = this->get_parameter("left_pwm_pin").as_int();
        left_dir_pin_ = this->get_parameter("left_dir_pin").as_int();
        right_pwm_pin_ = this->get_parameter("right_pwm_pin").as_int();
        right_dir_pin_ = this->get_parameter("right_dir_pin").as_int();
        pwm_frequency_ = this->get_parameter("pwm_frequency_hz").as_int();
        wheel_separation_ = this->get_parameter(
            "wheel_separation_m").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius_m").as_double();
        watchdog_timeout_ms_ = this->get_parameter(
            "watchdog_timeout_ms").as_int();

        // Validate parameters
        if (wheel_separation_ <= 0.0) {
            throw std::runtime_error("wheel_separation_m must be > 0");
        }
        if (wheel_radius_ <= 0.0) {
            throw std::runtime_error("wheel_radius_m must be > 0");
        }
        if (watchdog_timeout_ms_ <= 0) {
            throw std::runtime_error("watchdog_timeout_ms must be > 0");
        }

        // Initialise pigpio
        // TODO: pigpio may already be initialised if pigpiod is running.
        //       Consider using pigpio daemon interface (pigpiod_if2) instead
        //       of direct library init for better multi-process support.
        if (gpioInitialise() < 0) {
            throw std::runtime_error("Failed to initialise pigpio");
        }

        // Configure GPIO pins
        gpioSetMode(left_pwm_pin_, PI_OUTPUT);
        gpioSetMode(left_dir_pin_, PI_OUTPUT);
        gpioSetMode(right_pwm_pin_, PI_OUTPUT);
        gpioSetMode(right_dir_pin_, PI_OUTPUT);

        gpioSetPWMfrequency(left_pwm_pin_, pwm_frequency_);
        gpioSetPWMfrequency(right_pwm_pin_, pwm_frequency_);

        // Start with motors stopped
        stopMotors();

        // Create subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorController::cmdVelCallback, this,
                std::placeholders::_1));

        // Create event publisher
        pub_events_ = this->create_publisher<std_msgs::msg::String>(
            "/robot/events", 10);

        // Create watchdog timer
        auto watchdog_period = std::chrono::milliseconds(
            watchdog_timeout_ms_);
        watchdog_timer_ = this->create_wall_timer(
            watchdog_period,
            std::bind(&MotorController::watchdogCallback, this));

        last_cmd_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "motor_controller started — L_PWM:%d L_DIR:%d R_PWM:%d "
            "R_DIR:%d freq:%dHz watchdog:%dms",
            left_pwm_pin_, left_dir_pin_,
            right_pwm_pin_, right_dir_pin_,
            pwm_frequency_, watchdog_timeout_ms_);
    }

    ~MotorController() override
    {
        stopMotors();
        gpioTerminate();
    }

private:
    /**
     * @brief Convert Twist to differential drive and set motor PWM.
     */
    void cmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_cmd_time_ = this->now();

        double linear = msg->linear.x;
        double angular = msg->angular.z;

        // Differential drive kinematics
        double left_vel = (linear - angular * wheel_separation_ / 2.0)
            / wheel_radius_;
        double right_vel = (linear + angular * wheel_separation_ / 2.0)
            / wheel_radius_;

        setMotor(left_pwm_pin_, left_dir_pin_, left_vel);
        setMotor(right_pwm_pin_, right_dir_pin_, right_vel);
        motors_stopped_ = false;
    }

    /**
     * @brief Watchdog callback — stops motors if no cmd_vel received
     *        within timeout period.
     */
    void watchdogCallback()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        auto elapsed = (this->now() - last_cmd_time_).seconds() * 1000.0;
        if (elapsed > static_cast<double>(watchdog_timeout_ms_)) {
            if (!motors_stopped_) {
                RCLCPP_WARN(this->get_logger(),
                    "Watchdog triggered — no cmd_vel for %.0f ms, "
                    "stopping motors", elapsed);
                stopMotors();

                auto event = std_msgs::msg::String();
                event.data = "WATCHDOG_STOP";
                pub_events_->publish(event);
                motors_stopped_ = true;
            }
        }
    }

    /**
     * @brief Set a single motor's speed and direction via pigpio PWM.
     * @param pwm_pin  GPIO pin for PWM output.
     * @param dir_pin  GPIO pin for direction.
     * @param velocity Normalised velocity (-1.0 to 1.0 range after clamping).
     */
    void setMotor(int pwm_pin, int dir_pin, double velocity)
    {
        // Clamp to max range
        double max_vel = 10.0;  // rad/s — reasonable max for small motors
        double clamped = std::clamp(velocity, -max_vel, max_vel);
        double normalised = std::abs(clamped) / max_vel;

        // Direction: HIGH = forward, LOW = reverse
        gpioWrite(dir_pin, (clamped >= 0) ? 1 : 0);

        // PWM duty cycle 0–255
        int duty = static_cast<int>(normalised * 255.0);
        duty = std::clamp(duty, 0, 255);
        gpioPWM(pwm_pin, duty);
    }

    /**
     * @brief Stop all motors immediately.
     */
    void stopMotors()
    {
        gpioPWM(left_pwm_pin_, 0);
        gpioPWM(right_pwm_pin_, 0);
    }

    // Parameters
    int left_pwm_pin_;
    int left_dir_pin_;
    int right_pwm_pin_;
    int right_dir_pin_;
    int pwm_frequency_;
    double wheel_separation_;
    double wheel_radius_;
    int watchdog_timeout_ms_;

    // State
    std::mutex mutex_;
    rclcpp::Time last_cmd_time_;
    bool motors_stopped_;

    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_events_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<MotorController>());
    } catch (const std::runtime_error & e) {
        RCLCPP_FATAL(rclcpp::get_logger("motor_controller"),
            "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
