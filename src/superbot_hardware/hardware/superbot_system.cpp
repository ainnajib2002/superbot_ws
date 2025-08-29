#include "include/superbot_hardware/superbot_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>
#include <rclcpp/rclcpp.hpp>  // Include ROS2 logging
#include <pluginlib/class_list_macros.hpp> // Required for PLUGINLIB_EXPORT_CLASS

namespace superbot_hardware
{

  hardware_interface::return_type SuperbotHardware::configure(const hardware_interface::HardwareInfo & info)
  {
    RCLCPP_INFO(rclcpp::get_logger("SuperbotHardware"), "Configuring SuperbotHardware...");
  
    if (configure_default(info) != hardware_interface::return_type::OK)
    {
      RCLCPP_ERROR(rclcpp::get_logger("SuperbotHardware"), "Configuration failed. Default configuration error.");
      return hardware_interface::return_type::ERROR;
    }
  
    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  
    if (info_.hardware_parameters.count("pid_p") > 0) {
      cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
      cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
      cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
      cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
    }
  
    wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
    wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
  
    // ✅ Pindahkan ke sini
    RCLCPP_INFO(rclcpp::get_logger("SuperbotHardware"), "Configuration successful.");
    return hardware_interface::return_type::OK;
  }  // ← Sekarang penutup fungsi sudah benar

std::vector<hardware_interface::StateInterface> SuperbotHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("SuperbotHardware"), "Exporting state interfaces for wheels...");

  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Menambahkan state interfaces untuk posisi dan kecepatan roda
  state_interfaces.emplace_back(wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos);
  state_interfaces.emplace_back(wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel);

  state_interfaces.emplace_back(wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos);
  state_interfaces.emplace_back(wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel);

  RCLCPP_DEBUG(rclcpp::get_logger("SuperbotHardware"), "State interfaces exported.");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SuperbotHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("SuperbotHardware"), "Exporting command interfaces for wheels...");

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Menambahkan command interfaces untuk roda kiri dan kanan
  command_interfaces.emplace_back(wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd);
  command_interfaces.emplace_back(wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd);

  RCLCPP_DEBUG(rclcpp::get_logger("SuperbotHardware"), "Command interfaces exported.");
  return command_interfaces;
}

hardware_interface::return_type SuperbotHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("SuperbotHardware"), "Starting SuperbotHardware...");

  if (!comms_.connected())
  {
      if (!comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms))
      {
          RCLCPP_ERROR(rclcpp::get_logger("SuperbotHardware"), "Failed to connect to ESP32.");
          return hardware_interface::return_type::ERROR;
      }
  }

  if (!comms_.connected())
  {
      return hardware_interface::return_type::ERROR;
  }

  if (cfg_.pid_p > 0)
  {
      comms_.set_pid_values(cfg_.pid_p, cfg_.pid_d, cfg_.pid_i, cfg_.pid_o);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SuperbotHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("SuperbotHardware"), "Stopping SuperbotHardware...");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SuperbotHardware::read()
{
  RCLCPP_INFO(rclcpp::get_logger("SuperbotHardware"), "Reading encoder values...");

  if (!comms_.connected())
  {
      return hardware_interface::return_type::ERROR;
  }

  comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

  // Gunakan rclcpp::Clock untuk mendapatkan waktu delta
  static rclcpp::Clock clock;
  rclcpp::Time now = clock.now();
  double delta_seconds = (now - last_time_).seconds();
  last_time_ = now;

  // Perbaiki penggunaan pos_prev menjadi pos_prev_l dan pos_prev_r yang terpisah
  double pos_prev_l = wheel_l_.pos;
  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_l_.vel = (wheel_l_.pos - pos_prev_l) / delta_seconds;

  double pos_prev_r = wheel_r_.pos; // Tambahkan ini
  wheel_r_.pos = wheel_r_.calc_enc_angle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev_r) / delta_seconds; // Gunakan pos_prev_r

  // Hitung RPM
  // double wheel_l_rpm = wheel_l_.vel * (30.0 / M_PI);
  // double wheel_r_rpm = wheel_r_.vel * (30.0 / M_PI);
 
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SuperbotHardware::write()
{
  RCLCPP_INFO(rclcpp::get_logger("SuperbotHardware"), "Writing motor commands...");

  if (!comms_.connected())
  {
      return hardware_interface::return_type::ERROR;
  }

  int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
  int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
  comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);

  return hardware_interface::return_type::OK;
}

// hardware_interface::return_type SuperbotHardware::cleanup()
// {
//   RCLCPP_INFO(rclcpp::get_logger("SuperbotHardware"), "Cleaning up SuperbotHardware...");

//   if (comms_.connected())
//   {
//       comms_.disconnect();
//       RCLCPP_INFO(rclcpp::get_logger("SuperbotHardware"), "Communication disconnected.");
//   }

//   return hardware_interface::return_type::OK;
// }

// Deklarasikan kelas agar plugin dapat dikenali oleh ROS2
PLUGINLIB_EXPORT_CLASS(superbot_hardware::SuperbotHardware, hardware_interface::SystemInterface)

}  // namespace superbot_hardware
