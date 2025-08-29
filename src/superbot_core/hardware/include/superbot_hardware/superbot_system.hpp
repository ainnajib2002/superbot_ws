#ifndef SUPERBOT_HARDWARE__SUPERBOT_SYSTEM_HPP_
#define SUPERBOT_HARDWARE__SUPERBOT_SYSTEM_HPP_

// C++ Standard Library
#include <memory>
#include <string>
#include <vector>
#include <array>  // Untuk menyimpan roda secara terstruktur jika jumlah tetap

// ROS 2 Hardware Interface
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/handle.hpp"

// ROS 2 Logging
#include "rclcpp/rclcpp.hpp"

// Komponen lokal
#include "esp32_comms.hpp"
#include "wheel.hpp"

// Export visibility untuk shared library
#include "visibility_control.h"

namespace superbot_hardware
{

/**
 * @brief Kelas utama hardware interface untuk robot Omni berbasis ESP32.
 */
class SuperbotHardware
  : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  SuperbotHardware() = default;
  ~SuperbotHardware() override = default;

  /**
   * @brief Konfigurasi awal hardware dari parameter YAML.
   */
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Menyalakan hardware (dipanggil sebelum kontrol loop dimulai).
   */
  hardware_interface::return_type start() override;

  /**
   * @brief Mematikan hardware (dipanggil saat node dihentikan).
   */
  hardware_interface::return_type stop() override;

  /**
   * @brief Membaca data dari ESP32 (encoder).
   */
  hardware_interface::return_type read() override;

  /**
   * @brief Mengirim perintah ke ESP32 (PWM motor).
   */
  hardware_interface::return_type write() override;

  /**
   * @brief Mengirim perintah ke ESP32 (PWM motor).
   */
  // hardware_interface::return_type cleanup() override;
  /**
   * @brief Mengekspor interface posisi encoder ke controller ROS.
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Mengekspor interface command velocity ke controller ROS.
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  /**
   * @brief Struktur konfigurasi hardware yang dibaca dari YAML.
   */
  struct Config
  {
    std::string left_wheel_name;
    std::string right_wheel_name;
    float loop_rate = 0.0;
    std::string device;
    int baud_rate = 0;
    int timeout_ms = 0;
    int enc_counts_per_rev = 0;
    int pid_p = 0;
    int pid_d = 0;
    int pid_i = 0;
    int pid_o = 0;
  };

  /**
   * @brief Helper internal untuk parsing konfigurasi YAML.
   */
  hardware_interface::return_type init(const hardware_interface::HardwareInfo & info);

  /**
   * @brief Setup komunikasi serial dan PID motor.
   */
  bool configure_driver();

  /**
   * @brief Aktivasi hardware secara internal.
   */
  bool activate_driver();

  /**
   * @brief Menonaktifkan driver ESP32.
   */
  bool deactivate_driver();

  /**
   * @brief Cleanup resource setelah stop().
   */
  bool cleanup_driver();

  Esp32Comms comms_;   ///< Objek komunikasi serial ke ESP32
  Config cfg_;         ///< Konfigurasi hardware dari YAML
  Wheel wheel_l_;      ///< Objek roda kiri
  Wheel wheel_r_;      ///< Objek roda kanan
  rclcpp::Time last_time_; /// untuk simpan variabel
};

}  // namespace SUPERBOT_HARDWARE

#endif  // SUPERBOT_HARDWARE__SUPERBOT_SYSTEM_HPP_
