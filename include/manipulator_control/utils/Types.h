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

#endif // CONTROL_WORD_H
