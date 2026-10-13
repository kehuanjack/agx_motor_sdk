#pragma once

#include <memory>

#include <pybind11/pybind11.h>

namespace agx::motor {

class CanPort;

void BindCan(pybind11::module& m);
std::shared_ptr<CanPort> WrapCanPortObject(pybind11::object port);

}  // namespace agx::motor
