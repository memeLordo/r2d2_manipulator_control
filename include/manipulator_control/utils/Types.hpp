#ifndef R2D2_CONTROL_WORD_HPP
#define R2D2_CONTROL_WORD_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace r2d2_commands {
enum class ControlType : uint16_t {
  HOLD = 0x00,
  CONTROL_SPEED = 0x07,
  CONTROL_ANGLE = 0x0A,
  CHECK_ID = 0x800
};
}  // namespace r2d2_commands

namespace r2d2_state {
enum class NozzleType : uint8_t { NONE = 0, BRUSH, EMA };
enum class WorkMode : uint8_t { NONE = 0, MANUAL, AUTO, STOP = 0x80 };
enum class LockStatus : uint8_t { NONE = 0, LOCKED, UNLOCKED };

template <typename E>
struct EnumPair {
  E type{};
  std::string key{};

  template <typename T>
  void updateType(const T value) {
    type = static_cast<E>(value);
    updateKey();
  };
  void updateKey();
};
using NozzleTypePair = EnumPair<NozzleType>;
using WorkModePair = EnumPair<WorkMode>;
using LockStatusPair = EnumPair<LockStatus>;

template <>
inline void WorkModePair::updateKey() {
  switch (type) {
    case WorkMode::AUTO:
    case WorkMode::MANUAL:
    case WorkMode::STOP:
      key = "none";
      break;
    default:
      key = "";
  }
};
template <>
inline void LockStatusPair::updateKey() {
  switch (type) {
    case LockStatus::LOCKED:
    case LockStatus::UNLOCKED:
      key = "none";
      break;
    default:
      key = "";
  }
};
template <>
inline void NozzleTypePair::updateKey() {
  switch (type) {
    case NozzleType::BRUSH:
      key = "brush";
      break;
    case NozzleType::EMA:
      key = "ema";
      break;
    default:
      key = "";
  }
};
}  // namespace r2d2_state

namespace r2d2_type {
template <typename T, typename T1, typename T2>
struct pipebase_t {
  T1 diameter{};
  T2 thickness{};
  T radius() const { return (T)diameter / T{2} - (T)thickness; };
};
template <typename T1>
struct payloadbase_t {
  T1 force{};
};
template <typename T, typename T1>
struct manipulatorbase_t {
  T1 force_needed{};
  T1 force_tolerance{};
  T r0{};
};
template <typename T, typename T1>
struct jointbase_t {
  T omega{};
  T theta{};
  T1 control_word{};
};

namespace callback {
template <typename T>
using pipe_t = pipebase_t<T, uint16_t, uint8_t>;
template <typename T>
using manipulator16_t = manipulatorbase_t<T, int16_t>;
template <typename T, typename T1 = r2d2_commands::ControlType>
using joint_t = jointbase_t<T, T1>;

typedef payloadbase_t<int16_t> payload16_t;
typedef jointbase_t<int16_t, uint16_t> joint16_t;
}  // namespace callback

namespace config {
template <typename T>
struct joint_t {
  T length{};
  T speed{};
  T angle_offset{};
  T angle_tolerance{};
  std::vector<T> coeffs{};
};
template <typename T>
using manipulator_t = callback::manipulator16_t<T>;
}  // namespace config

}  // namespace r2d2_type
#endif  // R2D2_CONTROL_WORD_HPP
