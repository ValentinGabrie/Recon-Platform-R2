/**
 * @file esp32_sensor_node.cpp
 * @brief ESP32 I2C sensor driver node — reads ultrasonic distance data from
 *        ESP32 coprocessor over I2C and publishes sensor_msgs/msg/Range.
 *
 * Real-time C++17 node. Polls ESP32 registers at 10 Hz via WallTimer.
 */

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

/**
 * @class Esp32SensorNode
 * @brief ROS2 node that communicates with ESP32 over I2C to read ultrasonic
 *        sensor distances and publish them as Range messages.
 */
class Esp32SensorNode : public rclcpp::Node
{
public:
    Esp32SensorNode()
    : Node("esp32_sensor_node"),
      i2c_fd_(-1),
      sensor_read_errors_(0)
    {
        // Declare parameters — all configurable, no hardcoded values
        this->declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
        this->declare_parameter<int>("slave_address", 0x42);
        this->declare_parameter<double>("poll_rate_hz", 10.0);
        this->declare_parameter<double>("health_rate_hz", 1.0);
        this->declare_parameter<int>("consecutive_error_threshold", 10);

        // Retrieve parameters
        i2c_bus_ = this->get_parameter("i2c_bus").as_string();
        slave_address_ = static_cast<uint8_t>(
            this->get_parameter("slave_address").as_int());
        poll_rate_hz_ = this->get_parameter("poll_rate_hz").as_double();
        health_rate_hz_ = this->get_parameter("health_rate_hz").as_double();
        error_threshold_ = this->get_parameter(
            "consecutive_error_threshold").as_int();

        // Validate parameters
        if (poll_rate_hz_ <= 0.0) {
            throw std::runtime_error("poll_rate_hz must be > 0");
        }
        if (health_rate_hz_ <= 0.0) {
            throw std::runtime_error("health_rate_hz must be > 0");
        }
        if (error_threshold_ <= 0) {
            throw std::runtime_error(
                "consecutive_error_threshold must be > 0");
        }

        // Open I2C bus
        i2c_fd_ = open(i2c_bus_.c_str(), O_RDWR);
        if (i2c_fd_ < 0) {
            throw std::runtime_error(
                "Failed to open I2C bus: " + i2c_bus_);
        }

        // Set slave address
        if (ioctl(i2c_fd_, I2C_SLAVE, slave_address_) < 0) {
            close(i2c_fd_);
            throw std::runtime_error(
                "Failed to set I2C slave address: 0x" +
                std::to_string(slave_address_));
        }

        // Verify ESP32 firmware version (register 0xFF)
        try {
            uint16_t version = readSensorMm(i2c_fd_, 0xFF);
            uint8_t major = static_cast<uint8_t>(version >> 8);
            uint8_t minor = static_cast<uint8_t>(version & 0xFF);
            RCLCPP_INFO(this->get_logger(),
                "ESP32 firmware version: %d.%d", major, minor);
        } catch (const std::runtime_error & e) {
            close(i2c_fd_);
            throw std::runtime_error(
                "ESP32 firmware handshake failed: " +
                std::string(e.what()));
        }

        // Create publishers
        pub_front_ = this->create_publisher<sensor_msgs::msg::Range>(
            "/sensors/front", 10);
        pub_left_ = this->create_publisher<sensor_msgs::msg::Range>(
            "/sensors/left", 10);
        pub_right_ = this->create_publisher<sensor_msgs::msg::Range>(
            "/sensors/right", 10);
        pub_health_ = this->create_publisher<std_msgs::msg::UInt8>(
            "/sensors/health", 10);
        pub_events_ = this->create_publisher<std_msgs::msg::String>(
            "/robot/events", 10);

        // Create timers (never use sleep_for in callbacks)
        auto poll_period = std::chrono::duration<double>(
            1.0 / poll_rate_hz_);
        poll_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                poll_period),
            std::bind(&Esp32SensorNode::pollSensorsCallback, this));

        auto health_period = std::chrono::duration<double>(
            1.0 / health_rate_hz_);
        health_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                health_period),
            std::bind(&Esp32SensorNode::healthCallback, this));

        RCLCPP_INFO(this->get_logger(),
            "esp32_sensor_node started — I2C bus: %s, address: 0x%02X, "
            "poll rate: %.1f Hz",
            i2c_bus_.c_str(), slave_address_, poll_rate_hz_);
    }

    ~Esp32SensorNode() override
    {
        if (i2c_fd_ >= 0) {
            close(i2c_fd_);
        }
    }

private:
    /**
     * @brief Read a 2-byte big-endian uint16 from an ESP32 register.
     * @param fd   Open file descriptor for the I2C bus.
     * @param reg  Register address byte to query.
     * @return Distance in millimetres, or 0xFFFF on sensor error.
     * @throws std::runtime_error on I2C read failure.
     */
    static uint16_t readSensorMm(int fd, uint8_t reg)
    {
        if (write(fd, &reg, 1) != 1) {
            throw std::runtime_error("I2C register write failed");
        }
        uint8_t buf[2] = {0, 0};
        if (read(fd, buf, 2) != 2) {
            throw std::runtime_error("I2C data read failed");
        }
        return static_cast<uint16_t>((buf[0] << 8) | buf[1]);
    }

    /**
     * @brief Create a Range message for a given sensor reading.
     */
    sensor_msgs::msg::Range createRangeMsg(
        const std::string & frame_id, uint16_t distance_mm) const
    {
        sensor_msgs::msg::Range msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id;
        msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        msg.field_of_view = 0.26f;
        msg.min_range = 0.02f;
        msg.max_range = 4.0f;

        if (distance_mm == 0xFFFF) {
            msg.range = std::numeric_limits<float>::infinity();
            RCLCPP_DEBUG(this->get_logger(),
                "Sensor %s returned 0xFFFF (inf)", frame_id.c_str());
        } else {
            msg.range = static_cast<float>(distance_mm) / 1000.0f;
        }

        return msg;
    }

    /**
     * @brief Timer callback — polls all three sensor registers at poll_rate_hz.
     */
    void pollSensorsCallback()
    {
        try {
            uint16_t front = readSensorMm(i2c_fd_, 0x01);
            uint16_t left = readSensorMm(i2c_fd_, 0x02);
            uint16_t right = readSensorMm(i2c_fd_, 0x03);

            pub_front_->publish(
                createRangeMsg("ultrasonic_front", front));
            pub_left_->publish(
                createRangeMsg("ultrasonic_left", left));
            pub_right_->publish(
                createRangeMsg("ultrasonic_right", right));

            // Reset consecutive error counter on success
            sensor_read_errors_ = 0;
        } catch (const std::runtime_error & e) {
            sensor_read_errors_++;
            RCLCPP_WARN(this->get_logger(),
                "I2C read failed (consecutive: %d): %s",
                sensor_read_errors_, e.what());

            if (sensor_read_errors_ >= error_threshold_) {
                RCLCPP_ERROR(this->get_logger(),
                    "Consecutive I2C errors (%d) exceeded threshold "
                    "(%d) — publishing FAULT",
                    sensor_read_errors_, error_threshold_);

                auto event = std_msgs::msg::String();
                event.data = "SENSOR_FAULT";
                pub_events_->publish(event);
            }
        }
    }

    /**
     * @brief Timer callback — reads health register at health_rate_hz.
     */
    void healthCallback()
    {
        try {
            uint16_t status = readSensorMm(i2c_fd_, 0x04);
            auto msg = std_msgs::msg::UInt8();
            msg.data = static_cast<uint8_t>(status & 0xFF);
            pub_health_->publish(msg);
        } catch (const std::runtime_error & e) {
            RCLCPP_WARN(this->get_logger(),
                "Failed to read health register: %s", e.what());
        }
    }

    // I2C state
    int i2c_fd_;
    std::string i2c_bus_;
    uint8_t slave_address_;
    double poll_rate_hz_;
    double health_rate_hz_;
    int error_threshold_;
    int sensor_read_errors_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_front_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_left_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_right_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_health_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_events_;

    // Timers
    rclcpp::TimerBase::SharedPtr poll_timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<Esp32SensorNode>());
    } catch (const std::runtime_error & e) {
        RCLCPP_FATAL(rclcpp::get_logger("esp32_sensor_node"),
            "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
