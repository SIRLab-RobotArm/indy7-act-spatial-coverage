#pragma once

#include <cstdint>
#include <string>

namespace pipet_hand_mark7_driver
{

/// POSIX 시리얼 포트 래퍼
///
/// Mark7의 Tx(바이너리 write)와 Rx(ASCII line read)를
/// 단일 포트(/dev/ttyACM0)에서 처리한다.
class SerialPort
{
public:
  static constexpr int DEFAULT_BAUD = 115200;
  // The ros2_control loop runs at 20 Hz (50 ms period). Keep serial waiting
  // well below that budget so a missing status packet cannot stall commands.
  static constexpr int DEFAULT_TIMEOUT_MS = 10;

  explicit SerialPort(
    const std::string & device_path = "/dev/ttyACM0",
    int baud_rate = DEFAULT_BAUD);

  ~SerialPort();

  // 복사 금지, 이동 허용
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;
  SerialPort(SerialPort && other) noexcept;
  SerialPort & operator=(SerialPort && other) noexcept;

  /// 포트 열기 및 termios 설정 (115200, 8-N-1, raw)
  /// @return 성공 시 true
  bool open();

  /// 포트 닫기
  void close();

  bool is_open() const;

  /// 바이너리 데이터 전송 (Tx 패킷용)
  /// @return 성공 시 true
  bool write(const uint8_t * data, std::size_t len);

  /// \r\n 종료 ASCII 라인 수신 (Rx 패킷용)
  /// @param timeout_ms  타임아웃 (ms). 초과 시 빈 문자열 반환.
  /// @return  수신된 라인 (\r\n 포함). 타임아웃이면 빈 문자열.
  std::string read_crlf_line(int timeout_ms = DEFAULT_TIMEOUT_MS);

  /// 하위 호환용 (\n 종료 기준)
  /// TODO: 호출부를 모두 read_crlf_line()으로 전환 후 제거 가능
  std::string read_line(int timeout_ms = DEFAULT_TIMEOUT_MS);

private:
  std::string device_path_;
  int baud_rate_;
  int fd_;  ///< 파일 디스크립터 (-1 = 닫힘)
  std::string rx_buffer_;  ///< 제어 주기를 넘겨 보존하는 부분 수신 데이터
};

}  // namespace pipet_hand_mark7_driver
