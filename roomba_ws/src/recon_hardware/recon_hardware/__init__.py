"""recon_hardware — Pi-side hardware adapters for Recon-Platform-R2.

Holds the Python-side companions to the C++ hardware nodes:
  * framing             — ESP32 UART binary-frame parser + CRC8
  * esp32_uart_bridge   — ROS2 node that reads /dev/ttyUSB0 and republishes
                          IMU samples + button events as ROS topics.

The C++ side (sim_sensor_node) is built via ament_cmake in the same
package; the Python module is installed via ament_python_install_package.
"""
