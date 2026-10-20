#pragma once

#include <pybind11/pybind11.h>

namespace agx::motor::mechanism {

void BindMechanism(pybind11::module& m);

}  // namespace agx::motor::mechanism
