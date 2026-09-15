#pragma once

#include <array>
#include <cstdint>

namespace pipet_hand_mark7_driver
{

/// Mark7 손가락 인덱스
enum class FingerIndex : uint8_t
{
  THUMB_FLEX = 0,
  INDEX      = 1,
  MIDDLE     = 2,
  RING       = 3,
  LITTLE     = 4,
  THUMB_AB   = 5,
};

/// Mark7 방향 명령
enum class Direction : uint8_t
{
  IDLE    = 0,  ///< 아무 동작 없음
  FORWARD = 1,  ///< 전진. 목표값 > 현재면 전진, < 현재면 자동 역방향
  REVERSE = 2,  ///< 후진 (직접 역방향 지정)
  RESET   = 3,  ///< Position Counter를 0으로 초기화
};

/// PC → Mark7 송신 패킷 (11 bytes)
///
/// 패킷 구조:
///   B0  : Hand    (0xFD=Left, 0xFE=Right, 0xFF=Both)
///   B1  : Fingers bitmask (1=Thumb, 2=Index, 4=Middle, 8=Ring, 16=Little, 32=ThumbAbd)
///   B2  : Speed   (raw, 실제RPM = Data×200, 범위 30~150, 0=무시)
///   B3  : Current (raw, 실제mA = 600+Data×3, 0=무시)
///   B4  : Thumb Flex  목표 위치 (raw, 실제steps = -21+Data×2, 0=무시)
///   B5  : Index        목표 위치 (동일)
///   B6  : Middle       목표 위치 (동일)
///   B7  : Ring         목표 위치 (동일)
///   B8  : Little       목표 위치 (동일)
///   B9  : Thumb Abd    목표 위치 (동일)
///   B10 : Direction
///
/// 주의:
///   - position[i]=0 이면 해당 손가락 목표 위치 변경 없음 (0으로 이동이 아님)
///   - speed=0, current=0 이면 해당 파라미터 무시
///   - direction=RESET 이면 position counter가 0으로 초기화됨
///   - 헤더 및 XOR 체크섬 없음
struct Mark7TxPacket
{
  static constexpr uint8_t HAND_LEFT  = 0xFD;
  static constexpr uint8_t HAND_RIGHT = 0xFE;
  static constexpr uint8_t HAND_BOTH  = 0xFF;

  /// steps → position raw byte 변환: raw = (steps + 21) / 2
  /// raw=0 은 "변경 없음" 특수값이므로 steps=-21 이하는 1로 클램프
  static uint8_t steps_to_raw(int steps)
  {
    const int raw = (steps + 21) / 2;
    if (raw <= 0) { return 1; }    // 0은 "무시" 예약값 → 최솟값 1
    if (raw > 255) { return 255; }
    return static_cast<uint8_t>(raw);
  }

  uint8_t   hand        = HAND_BOTH;  ///< 손 선택
  uint8_t   finger_mask = 0x3F;       ///< 손가락 비트마스크 (기본: 전체 6개)
  uint8_t   speed       = 0;          ///< raw (0=무시, 실제RPM=Data×200)
  uint8_t   current     = 0;          ///< raw (0=무시, 실제mA=600+Data×3)
  uint8_t   position[6] = {0};        ///< 손가락별 raw 위치 (0=무시, FingerIndex 순서)
  Direction direction   = Direction::IDLE;

  /// 11바이트 패킷으로 인코딩
  std::array<uint8_t, 11> encode() const
  {
    std::array<uint8_t, 11> buf = {};
    buf[0]  = hand;
    buf[1]  = finger_mask;
    buf[2]  = speed;
    buf[3]  = current;
    buf[4]  = position[0];  // Thumb Flex
    buf[5]  = position[1];  // Index
    buf[6]  = position[2];  // Middle
    buf[7]  = position[3];  // Ring
    buf[8]  = position[4];  // Little
    buf[9]  = position[5];  // Thumb Abd
    buf[10] = static_cast<uint8_t>(direction);
    return buf;
  }
};

}  // namespace pipet_hand_mark7_driver
