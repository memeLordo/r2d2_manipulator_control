#ifndef R2D2_CONTROL_WORD_HPP
#define R2D2_CONTROL_WORD_HPP

#include <cstdint>

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

namespace r2d2_type {

template <typename T1, typename T2, typename T> struct pipe_t {
  T1 diameter{};
  T2 thickness{};
  T radius() const { return (T)diameter / T{2} - (T)thickness; };
};

template <typename T1> struct payload_t {
  T1 force{};
};

template <typename T1, typename T> struct manipulator_t {
  T1 force_needed{};
  T r0{};
};

template <typename T, typename T2> struct joint_t {
  T omega{};
  T theta{};
  T2 control_word{};
};

template <typename T, typename T2> using elbow_t = joint_t<T, T2>;
template <typename T, typename T2> using shoulder_t = joint_t<T, T2>;

template <typename T> using upipe_t = pipe_t<uint16_t, uint8_t, T>;
template <typename T> using manipulator16_t = manipulator_t<int16_t, T>;

typedef payload_t<int16_t> payload16_t;
typedef joint_t<int16_t, uint16_t> joint16_t;
typedef joint16_t elbow16_t;
typedef joint16_t shoulder16_t;

} // namespace r2d2_type

#endif // R2D2_CONTROL_WORD_HPP
