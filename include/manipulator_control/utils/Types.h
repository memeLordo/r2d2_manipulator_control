#ifndef CONTROL_WORD_H
#define CONTROL_WORD_H

namespace r2d2_commands {

enum class ControlType : uint16_t {
  HOLD = 0x00,
  CONTROL_SPEED = 0x07,
  CONTROL_ANGLE = 0x0A,
  CHECK_ID = 0x800
};

} // namespace r2d2_commands

namespace r2d2_state {

enum class LockStatus : uint8_t { NONE = 0, LOCKED, UNLOCKED };
enum class WorkMode : uint8_t { NONE = 0, MANUAL, AUTO, SETUP };
enum class NozzleType : uint8_t { NONE = 0, BRUSH, EMA };

} // namespace r2d2_state

#endif // CONTROL_WORD_H
