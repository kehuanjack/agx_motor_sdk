#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "agx_motor_sdk/can/can_port.hpp"
#include "agx_motor_sdk/can/socket_can.hpp"

namespace py = pybind11;

namespace {

class PyCanPortWrapper {
 public:
  explicit PyCanPortWrapper(std::string channel, bool local_loopback = false)
      : port_(std::make_shared<agx::motor::SocketCanPort>(std::move(channel),
                                                          local_loopback)) {}

  bool open() { return port_->Open(); }
  void close() { port_->Close(); }
  bool is_open() const { return port_->IsOpen(); }

  void send(uint32_t can_id, py::bytes payload) {
    std::string data = payload;
    agx::motor::CanFrame frame;
    frame.id = can_id;
    frame.data.assign(data.begin(), data.end());
    port_->Send(frame);
  }

  void set_callback(py::object callback) {
    callback_ = std::move(callback);
    if (callback_.is_none()) {
      port_->SetReceiveHandler(nullptr);
      return;
    }
    port_->SetReceiveHandler([this](const agx::motor::CanFrame& frame) {
      py::gil_scoped_acquire acquire;
      callback_(frame.id, py::bytes(reinterpret_cast<const char*>(frame.data.data()),
                                    frame.data.size()));
    });
  }

 private:
  std::shared_ptr<agx::motor::SocketCanPort> port_;
  py::object callback_;
};

}  // namespace

PYBIND11_MODULE(_native, m) {
  m.doc() = "agx_motor_sdk native CAN backend";

  py::class_<PyCanPortWrapper>(m, "SocketCanPort")
      .def(py::init<std::string, bool>(), py::arg("channel") = "can0",
           py::arg("local_loopback") = false)
      .def("open", &PyCanPortWrapper::open)
      .def("close", &PyCanPortWrapper::close)
      .def("is_open", &PyCanPortWrapper::is_open)
      .def("send", &PyCanPortWrapper::send, py::arg("can_id"), py::arg("data"))
      .def("set_callback", &PyCanPortWrapper::set_callback);
}
