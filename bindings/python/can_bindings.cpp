#include "can_bindings.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>

#include "agx_motor_sdk/can/can_port.hpp"
#include "agx_motor_sdk/can/socket_can.hpp"

namespace py = pybind11;

namespace agx::motor {
namespace {

class PyCanPortWrapper {
 public:
  explicit PyCanPortWrapper(std::string channel, bool local_loopback = false)
      : port_(std::make_shared<SocketCanPort>(std::move(channel), local_loopback)) {}

  ~PyCanPortWrapper() {
    auto port = std::move(port_);
    auto cleanup = [port = std::move(port)]() {
      port->SetReceiveHandler(nullptr);
      port->Close();
    };
    if (PyGILState_Check()) {
      py::gil_scoped_release release;
      std::thread(std::move(cleanup)).detach();
    } else {
      std::thread(std::move(cleanup)).detach();
    }
  }

  bool open() { return port_->Open(); }
  void close() {
    auto port = port_;
    py::gil_scoped_release release;
    port->SetReceiveHandler(nullptr);
    port->Close();
  }
  bool is_open() const { return port_->IsOpen(); }

  void send(uint32_t can_id, py::bytes payload) {
    std::string data = payload;
    CanFrame frame;
    frame.id = can_id;
    frame.data.assign(data.begin(), data.end());
    port_->Send(frame);
  }

  void set_callback(py::object callback) {
    auto port = port_;
    CanReceiveHandler handler;
    if (!callback.is_none()) {
      auto owner = std::shared_ptr<py::object>(
          new py::object(std::move(callback)), [](py::object* object) {
            py::gil_scoped_acquire gil;
            delete object;
          });
      handler = [owner](const CanFrame& frame) {
        py::gil_scoped_acquire acquire;
        try {
          (*owner)(frame.id, py::bytes(reinterpret_cast<const char*>(frame.data.data()),
                                     frame.data.size()));
        } catch (py::error_already_set& error) {
          error.discard_as_unraisable("SocketCanPort receive callback");
        }
      };
    }
    py::gil_scoped_release release;
    port->SetReceiveHandler(std::move(handler));
  }

  std::shared_ptr<CanPort> can_port() const { return port_; }

 private:
  std::shared_ptr<SocketCanPort> port_;
};

class PyObjectCanPort : public CanPort, public std::enable_shared_from_this<PyObjectCanPort> {
 public:
  explicit PyObjectCanPort(py::object port) : port_(std::move(port)) {}

  bool Open() override {
    py::gil_scoped_acquire gil;
    if (py::hasattr(port_, "connect")) {
      port_.attr("connect")();
    }
    opened_.store(true);
    WirePythonCallback();
    return true;
  }

  void Close() override {
    py::gil_scoped_acquire gil;
    handler_ = nullptr;
    py_callback_adapter_ = py::none();
    opened_.store(false);
    if (py::hasattr(port_, "clear_callback")) {
      port_.attr("clear_callback")();
    }
    if (py::hasattr(port_, "close")) {
      port_.attr("close")();
    }
  }

  bool IsOpen() const override {
    py::gil_scoped_acquire gil;
    if (py::hasattr(port_, "is_connected")) {
      return port_.attr("is_connected")().cast<bool>();
    }
    if (py::hasattr(port_, "is_open")) {
      return port_.attr("is_open")().cast<bool>();
    }
    return opened_.load();
  }

  void Send(const CanFrame& frame) override {
    py::gil_scoped_acquire gil;
    py::module can_msg = py::module::import("can.message");
    py::object Message = can_msg.attr("Message");
    py::bytes payload(reinterpret_cast<const char*>(frame.data.data()), frame.data.size());
    py::object msg =
        Message(py::arg("arbitration_id") = frame.id, py::arg("data") = payload,
                py::arg("is_extended_id") = false);
    port_.attr("send")(msg);
  }

  void SetReceiveHandler(CanReceiveHandler handler) override {
    py::gil_scoped_acquire gil;
    handler_ = std::move(handler);
    WirePythonCallback();
  }

 private:
  void WirePythonCallback() {
    py::gil_scoped_acquire gil;
    if (!handler_) {
      if (py::hasattr(port_, "clear_callback")) {
        port_.attr("clear_callback")();
      }
      py_callback_adapter_ = py::none();
      return;
    }
    std::weak_ptr<PyObjectCanPort> weak_self = shared_from_this();
    py_callback_adapter_ = py::cpp_function([weak_self](py::object msg) {
      auto self = weak_self.lock();
      if (!self || !self->handler_) {
        return;
      }
      auto handler = self->handler_;
      CanFrame frame;
      frame.id = msg.attr("arbitration_id").cast<uint32_t>();
      const std::string payload = py::reinterpret_borrow<py::bytes>(msg.attr("data"));
      frame.data.assign(payload.begin(), payload.end());
      if (py::hasattr(msg, "timestamp")) {
        try {
          const double ts = msg.attr("timestamp").cast<double>();
          frame.timestamp_ns = static_cast<uint64_t>(ts * 1e9);
        } catch (...) {
        }
      }
      handler(frame);
    });
    port_.attr("set_callback")(py_callback_adapter_);
  }

  py::object port_;
  std::atomic<bool> opened_{false};
  CanReceiveHandler handler_;
  py::object py_callback_adapter_{py::none()};
};

}  // namespace

std::shared_ptr<CanPort> WrapCanPortObject(py::object port) {
  if (port.is_none()) {
    return nullptr;
  }
  if (py::isinstance<PyCanPortWrapper>(port)) {
    return port.cast<PyCanPortWrapper&>().can_port();
  }
  if (py::hasattr(port, "_port")) {
    py::object inner = port.attr("_port");
    if (py::isinstance<PyCanPortWrapper>(inner)) {
      return inner.cast<PyCanPortWrapper&>().can_port();
    }
  }
  return std::shared_ptr<CanPort>(new PyObjectCanPort(std::move(port)), [](CanPort* object) {
    py::gil_scoped_acquire gil;
    delete object;
  });
}

void BindCan(py::module& m) {
  py::class_<PyCanPortWrapper>(m, "SocketCanPort")
      .def(py::init<std::string, bool>(), py::arg("channel") = "can0",
           py::arg("local_loopback") = false)
      .def("open", &PyCanPortWrapper::open)
      .def("close", &PyCanPortWrapper::close)
      .def("is_open", &PyCanPortWrapper::is_open)
      .def("send", &PyCanPortWrapper::send, py::arg("can_id"), py::arg("data"))
      .def("set_callback", &PyCanPortWrapper::set_callback);

  m.def("wrap_can_port", &WrapCanPortObject, py::arg("port"));
}

}  // namespace agx::motor
