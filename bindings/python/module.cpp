#include <pybind11/pybind11.h>

#include "can_bindings.hpp"
#include "mechanism_bindings.hpp"
#include "motor_bindings.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_native, m) {
  m.doc() = "agx_motor_sdk native backend (CAN, motor drivers, mechanism)";

  agx::motor::BindCan(m);
  agx::motor::BindMotor(m);
  agx::motor::mechanism::BindMechanism(m);
}
