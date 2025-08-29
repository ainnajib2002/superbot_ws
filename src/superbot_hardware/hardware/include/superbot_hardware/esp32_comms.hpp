#ifndef SUPERBOT_HARDWARE_ESP32_COMMS_HPP
#define SUPERBOT_HARDWARE_ESP32_COMMS_HPP

#include <libserial/SerialPort.h>
#include <iostream>
#include <sstream>
#include <string>

/// @brief Helper untuk mengonversi int ke LibSerial::BaudRate.
inline LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
    switch (baud_rate)
    {
        case 1200:    return LibSerial::BaudRate::BAUD_1200;
        case 1800:    return LibSerial::BaudRate::BAUD_1800;
        case 2400:    return LibSerial::BaudRate::BAUD_2400;
        case 4800:    return LibSerial::BaudRate::BAUD_4800;
        case 9600:    return LibSerial::BaudRate::BAUD_9600;
        case 19200:   return LibSerial::BaudRate::BAUD_19200;
        case 38400:   return LibSerial::BaudRate::BAUD_38400;
        case 57600:   return LibSerial::BaudRate::BAUD_57600;
        case 115200:  return LibSerial::BaudRate::BAUD_115200;
        case 230400:  return LibSerial::BaudRate::BAUD_230400;
        default:
            std::cerr << "[Error] Unsupported baud rate: " << baud_rate 
                      << ". Defaulting to 115200.\n";
            return LibSerial::BaudRate::BAUD_115200;
    }
}

class Esp32Comms
{
public:
    Esp32Comms() = default;
    ~Esp32Comms() { disconnect(); }

    /// @brief Membuka koneksi serial.
    bool connect(const std::string& serial_device, int32_t baud_rate, int32_t timeout_ms)
    {  
        timeout_ms_ = timeout_ms;
        try
        {
            serial_conn_.Open(serial_device);
            serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
        }
        catch (const LibSerial::OpenFailed&)
        {
            std::cerr << "[Error] Failed to open serial device: " << serial_device << "\n";
            return false;
        }
        return serial_conn_.IsOpen();
    }

    /// @brief Menutup koneksi serial jika terbuka.
    void disconnect()
    {
        if (serial_conn_.IsOpen()) {
            serial_conn_.Close();
        }
    }

    /// @brief Mengecek apakah koneksi serial masih aktif.
    bool connected() const
    {
        return serial_conn_.IsOpen();
    }

    /// @brief Mengirim pesan ke ESP32 dan membaca balasannya.
    std::string send_msg(const std::string& msg_to_send, bool print_output = false)
    {
        if (!serial_conn_.IsOpen()) {
            std::cerr << "[Error] Serial connection not open!\n";
            return "";
        }

        serial_conn_.FlushIOBuffers();
        serial_conn_.Write(msg_to_send);

        std::string response;
        try
        {
            serial_conn_.ReadLine(response, '\n', timeout_ms_);
        }
        catch (const LibSerial::ReadTimeout&)
        {
            std::cerr << "[Warning] Read timeout.\n";
        }

        if (print_output)
        {
            std::cout << "[Serial] Sent: " << msg_to_send
                      << " | Recv: " << response << "\n";
        }

        return response;
    }

    /// @brief Mengirimkan karakter kosong (newline) ke ESP32.
    void send_empty_msg()
    {
        send_msg("\r");
    }

    /// @brief Membaca nilai dari dua encoder motor.
    void read_encoder_values(int& val_1, int& val_2)
    {
        std::string response = send_msg("e\r");
        std::istringstream iss(response);
        iss >> val_1 >> val_2;
    }

    /// @brief Mengatur nilai PWM motor kiri dan kanan.
    void set_motor_values(int val_1, int val_2)
    {
        std::ostringstream ss;
        ss << "m " << val_1 << " " << val_2 << "\r";
        send_msg(ss.str());
    }

    /// @brief Mengatur parameter PID.
    void set_pid_values(int k_p, int k_d, int k_i, int k_o)
    {
        std::ostringstream ss;
        ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
        send_msg(ss.str());
    }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_ = 1000;
};

#endif // SUPERBOT_HARDWARE_ESP32_COMMS_HPP
