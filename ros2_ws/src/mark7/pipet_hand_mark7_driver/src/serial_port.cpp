#include "pipet_hand_mark7_driver/serial_port.hpp"

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace pipet_hand_mark7_driver
{

SerialPort::SerialPort(const std::string & device_path, int baud_rate)
: device_path_(device_path), baud_rate_(baud_rate), fd_(-1)
{
}

SerialPort::~SerialPort()
{
  close();
}

SerialPort::SerialPort(SerialPort && other) noexcept
: device_path_(std::move(other.device_path_)),
  baud_rate_(other.baud_rate_),
  fd_(other.fd_),
  rx_buffer_(std::move(other.rx_buffer_))
{
  other.fd_ = -1;
}

SerialPort & SerialPort::operator=(SerialPort && other) noexcept
{
  if (this != &other) {
    close();
    device_path_ = std::move(other.device_path_);
    baud_rate_   = other.baud_rate_;
    fd_          = other.fd_;
    rx_buffer_   = std::move(other.rx_buffer_);
    other.fd_    = -1;
  }
  return *this;
}

bool SerialPort::open()
{
  fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  // termios 설정: 115200bps, 8-N-1, raw 모드
  struct termios tty;
  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  cfmakeraw(&tty);          // raw 모드 (echo 끄기, 특수문자 처리 끄기)

  tty.c_cflag |= (CLOCAL | CREAD);  // 로컬 연결, 수신 활성화
  tty.c_cflag &= ~CSTOPB;           // 스톱 비트 1
  tty.c_cflag &= ~CRTSCTS;          // 하드웨어 흐름 제어 끄기

  tty.c_cc[VMIN]  = 0;  // 최소 수신 바이트 수 (non-blocking)
  tty.c_cc[VTIME] = 0;  // 읽기 타임아웃 (select로 별도 처리)

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  tcflush(fd_, TCIOFLUSH);  // 버퍼 비우기
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  rx_buffer_.clear();
}

bool SerialPort::is_open() const
{
  return fd_ >= 0;
}

bool SerialPort::write(const uint8_t * data, std::size_t len)
{
  if (!is_open()) {
    return false;
  }

  std::size_t written = 0;
  while (written < len) {
    ssize_t n = ::write(fd_, data + written, len - written);
    if (n < 0) {
      return false;
    }
    written += static_cast<std::size_t>(n);
  }
  return true;
}

std::string SerialPort::read_crlf_line(int timeout_ms)
{
  if (!is_open()) {
    return {};
  }

  const auto take_complete_line = [this]() -> std::string {
      const auto end = rx_buffer_.find("\r\n");
      if (end == std::string::npos) {
        return {};
      }
      std::string line = rx_buffer_.substr(0, end + 2);
      rx_buffer_.erase(0, end + 2);
      return line;
    };

  if (auto line = take_complete_line(); !line.empty()) {
    return line;
  }

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(fd_, &read_fds);

  const auto timeout_us = static_cast<long>(timeout_ms) * 1000L;
  struct timeval tv;
  tv.tv_sec  = timeout_us / 1'000'000L;
  tv.tv_usec = timeout_us % 1'000'000L;

  const int ready = select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
  if (ready <= 0) {
    return {};
  }

  // Drain every byte currently available without waiting for the remainder
  // of a fragmented line. Partial data stays buffered for the next 50 ms
  // control cycle instead of blocking this cycle.
  char chunk[256];
  while (true) {
    const ssize_t count = ::read(fd_, chunk, sizeof(chunk));
    if (count > 0) {
      rx_buffer_.append(chunk, static_cast<std::size_t>(count));
      continue;
    }
    if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      rx_buffer_.clear();
    }
    break;
  }

  // Avoid unbounded growth if the device sends malformed data without CRLF.
  if (rx_buffer_.size() > 4096) {
    rx_buffer_.clear();
    return {};
  }
  return take_complete_line();
}

std::string SerialPort::read_line(int timeout_ms)
{
  return read_crlf_line(timeout_ms);
}

}  // namespace pipet_hand_mark7_driver
