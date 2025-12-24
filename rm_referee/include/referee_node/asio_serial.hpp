#pragma once

#include <boost/asio.hpp>

namespace asio_serial {

class Serial {
 public:
  Serial(const std::string& port, uint32_t baudrate, uint32_t timeout_ms = 1000)
      : io_context_(), serial_port_(io_context_), timeout_(timeout_ms) {
    serial_port_.open(port);
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_.set_option(
        boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  }

  ~Serial() {
    if (serial_port_.is_open()) {
      serial_port_.close();
    }
  }

  bool isOpen() const { return serial_port_.is_open(); }

  void close() {
    if (serial_port_.is_open()) {
      serial_port_.close();
    }
  }

  std::string read(size_t size = 1) {
    std::string result;
    result.resize(size);

    boost::system::error_code ec;
    size_t bytes_read = boost::asio::read(serial_port_, boost::asio::buffer(result), ec);

    if (ec) {
      result.clear();
    } else {
      result.resize(bytes_read);
    }

    return result;
  }

  size_t write(const std::string& data) {
    boost::system::error_code ec;
    size_t bytes_written = boost::asio::write(serial_port_, boost::asio::buffer(data), ec);

    if (ec) {
      return 0;
    }

    return bytes_written;
  }

 private:
  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;
  uint32_t timeout_;
};

}  // namespace asio_serial
