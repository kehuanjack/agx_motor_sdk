#pragma once

#include <pybind11/pybind11.h>

namespace agx::motor {

void BindMotor(pybind11::module& m);

}  // namespace agx::motor
