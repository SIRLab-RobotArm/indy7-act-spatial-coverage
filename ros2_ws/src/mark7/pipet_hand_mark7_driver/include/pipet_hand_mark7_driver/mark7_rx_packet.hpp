#pragma once

#include <array>
#include <cstdint>
#include <sstream>
#include <string>

namespace pipet_hand_mark7_driver
{

/// 손가락 한 개의 상태 (이미 스케일된 값)
struct FingerState
{
  int16_t position;     ///< Position count (절대값)
  int16_t current;      ///< 전류 (이미 스케일된 값)
  int16_t temperature;  ///< 온도 (°C)
};

/// Mark7 → PC 수신 패킷 (ASCII 형식)
///
/// 실제 수신 형식:
///   "R,139,1140,35,171,1140,34,203,1140,34,235,1140,34,267,1140,34,299,1140,34\r\n"
///
/// 구조: L|R, 손가락 6개 × (position, current, temperature), 쉼표 구분, \r\n 종료
/// 손가락 순서: Thumb Flex, Index, Middle, Ring, Little, Thumb Ab
///
/// 주의: 첫 번째 필드는 손 식별자(L/R)이므로 파싱 시 건너뜀.
struct Mark7RxPacket
{
  static constexpr int NUM_FINGERS = 6;
  static constexpr int VALUES_PER_FINGER = 3;
  static constexpr int TOTAL_VALUES = NUM_FINGERS * VALUES_PER_FINGER;

  FingerState fingers[NUM_FINGERS] = {};

  /// ASCII 라인을 파싱하여 Mark7RxPacket으로 변환
  ///
  /// @param line  수신된 ASCII 라인 ("\r\n" 포함 여부 무관)
  /// @param out   파싱 결과를 저장할 패킷
  /// @return      파싱 성공 시 true, 실패 시 false
  static bool parse(const std::string & line, Mark7RxPacket & out)
  {
    // 쉼표를 공백으로 치환하여 istringstream으로 파싱
    std::string buf = line;
    for (char & c : buf) {
      if (c == ',') { c = ' '; }
    }

    std::istringstream ss(buf);

    // 첫 번째 필드: 손 식별자 (L 또는 R) → 건너뜀
    std::string hand_id;
    if (!(ss >> hand_id)) {
      return false;
    }

    int values[TOTAL_VALUES];
    for (int i = 0; i < TOTAL_VALUES; ++i) {
      if (!(ss >> values[i])) {
        return false;  // 값 부족 또는 파싱 오류
      }
    }

    for (int i = 0; i < NUM_FINGERS; ++i) {
      out.fingers[i].position    = static_cast<int16_t>(values[i * VALUES_PER_FINGER + 0]);
      out.fingers[i].current     = static_cast<int16_t>(values[i * VALUES_PER_FINGER + 1]);
      out.fingers[i].temperature = static_cast<int16_t>(values[i * VALUES_PER_FINGER + 2]);
    }

    return true;
  }
};

}  // namespace pipet_hand_mark7_driver
