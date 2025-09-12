#include "ManipulatorControl.hpp"

using namespace r2d2_state;
using namespace r2d2_type;

template <typename T>
ManipulatorControlHandler<T>::ManipulatorControlHandler(ros::NodeHandle *node)
    : ManipulatorConfig<T>{},
      m_pipe{node},
      m_payload{node},
      m_elbow{node},
      m_shoulder{node} {
  const double RATE = node->param<T>("control_rate", 20);
  ROS_DEBUG_STREAM("Set RATE: " << RATE);
  m_timer = node->createTimer(
      ros::Duration(1 / RATE),
      &ManipulatorControlHandler<T>::callbackManipulator, this);
}
template <typename T>
void ManipulatorControlHandler<T>::callbackManipulator(
    const ros::TimerEvent &) {
  ROS_DEBUG_STREAM("\ncallbackManipulator()");
  /**
   * INFO:
   * 0. Получить данные для манипулятора (и трубы)
   * 1. Проверка автоматического разжатия
   * 2. Приём типа насадки (Щётка/ЕМА)
   * 3. Проверка статуса блокировки
   * 4. Обновить оставшиеся переменные
   * 5. Опубликовать все переменные
   */
  switch (m_workMode.type) {
    case WorkMode::AUTO:
      ROS_DEBUG_STREAM(YELLOW("WorkMode::AUTO"));
      processControl(m_pipe.getRadius(), m_payload.getForce());
      return;

    case WorkMode::STOP:
      ROS_DEBUG_STREAM(YELLOW("WorkMode::STOP"));
      processStop(getRadius());
      return;

    case WorkMode::MANUAL:
      ROS_DEBUG_STREAM(YELLOW("WorkMode::MANUAL"));
      this->resetMode();
      return;

    default:
      ROS_DEBUG_STREAM(YELLOW("Pending mode"));
      return;
  }
}
template <typename T>
void ManipulatorControlHandler<T>::checkSetup(const T radius) {
  if (!m_needsSetup) {
    ROS_DEBUG_STREAM(CYAN("Control radius reached!"));
    return;
  }
  ROS_DEBUG_STREAM(MAGENTA("\ncheckSetup()"));
  const bool hasRadiusReached_{getCurrentRadius() >= radius};
  ROS_DEBUG_STREAM(CYAN("hasRadiusReached_ = " << hasRadiusReached_));
  m_needsSetup = !hasRadiusReached_;

  ROS_DEBUG_STREAM(RED("\nend") << MAGENTA("::checkSetup()"));
}
template <typename T>
void ManipulatorControlHandler<T>::processStop(const T radius) {
  ROS_DEBUG_STREAM(YELLOW("WorkMode::STOP"));
  processAngleControl(radius);
  publishResults();
}
template <typename T>
void ManipulatorControlHandler<T>::processControl(const T radius,
                                                  const T force) {
  switch (m_lockStatus.type) {
    case LockStatus::UNLOCKED:
      ROS_DEBUG_STREAM(YELLOW("LockStatus::UNLOCKED"));
      checkSetup(radius);
      processAngleControl(radius);
      processForceControl(force);
      break;
    default:
      break;
  }
  publishResults();
}
template <typename T>
void ManipulatorControlHandler<T>::processAngleControl(const T radius) {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessRadiusControl()"));
  m_shoulder.updateAngleByRadius(radius);
  m_elbow.updateAngleByRadius(radius);
  ROS_DEBUG_STREAM(RED("\nend") << MAGENTA("::processRadiusControl()"));
}
template <typename T>
void ManipulatorControlHandler<T>::processForceControl(const T force) {
  if (m_needsSetup) return;
  ROS_DEBUG_STREAM(MAGENTA("\nprocessForceControl()"));
  m_elbow.updateAngleByDiff(checkForceDiff(force));
  ROS_DEBUG_STREAM(RED("\nend") << MAGENTA("::processForceControl()"));
}

template class ManipulatorControlHandler<>;
