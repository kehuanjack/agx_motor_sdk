#pragma once

#include "agx_motor_sdk/mechanism/motor_base.hpp"
#include "agx_motor_sdk/motor/rb200/rb200_driver.hpp"
#include "agx_motor_sdk/motor/rb200/types.hpp"

#include <memory>
#include <optional>

namespace agx::motor::mechanism {

/// RB200 joint motor: common commands plus RB200 low-speed feedback
/// (CiA402 status word, error code).
///
/// Created and shared by ``MotorRegistry``; reach it from
/// ``JointGroup::Motor`` via ``dynamic_cast<Rb200Motor*>``.
class Rb200Motor : public MotorBase {
 public:
  /// Cached RB200 status word / error code (non-blocking).
  std::optional<agx::motor::Rb200LowSpeedState> GetLowSpeedState() const;

 private:
  friend class MotorRegistry;

  using MotorBase::MotorBase;
};

}  // namespace agx::motor::mechanism
