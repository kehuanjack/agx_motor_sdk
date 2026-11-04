#include "agx_motor_sdk/can/can_port.hpp"
#include "agx_motor_sdk/mechanism/arm/arm_motor.hpp"
#include "agx_motor_sdk/mechanism/rb200/rb200_motor.hpp"
#include "agx_motor_sdk/motor/rb200/rb200_driver.hpp"
#include "agx_motor_sdk/mechanism/can_bus.hpp"
#include "agx_motor_sdk/mechanism/motor_registry.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>

using namespace agx::motor;
using namespace agx::motor::mechanism;

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

struct PortStats {
  int closed{0};
  int destroyed{0};
  int detached{0};
  int sent{0};
};

class FakeCanPort final : public CanPort {
 public:
  explicit FakeCanPort(std::shared_ptr<PortStats> stats) : stats_(std::move(stats)) {}
  ~FakeCanPort() override { ++stats_->destroyed; }
  bool Open() override { opened_ = true; return true; }
  void Close() override { opened_ = false; ++stats_->closed; }
  bool IsOpen() const override { return opened_; }
  void Send(const CanFrame&) override { ++stats_->sent; }
  void SetReceiveHandler(CanReceiveHandler handler) override {
    if (!handler) ++stats_->detached;
    handler_ = std::move(handler);
  }
  void Inject(const CanFrame& frame) {
    if (handler_) handler_(frame);
  }
 private:
  std::shared_ptr<PortStats> stats_;
  bool opened_{false};
  CanReceiveHandler handler_;
};

JointConfig Config() {
  JointConfig config;
  config.name = "yaw";
  config.node_id = 1;
  config.can_channel = "fake0";
  config.mit_tf = 8.f;
  config.mit_kp = 1.f;
  config.mit_kd = 1.f;
  return config;
}

void CheckExpired(MotorBase& base) {
  auto& motor = dynamic_cast<ArmMotor&>(base);
  Require(!motor.SetEnable(), "expired enable");
  Require(!motor.Configure(), "expired configure");
  Require(!motor.SetMit(0.f, 0.f, 0.f), "expired MIT");
  Require(!motor.SetTarget(0.f, 1, 0.f), "expired target");
  Require(!motor.SetTargetPosition(0.f), "expired position");
  Require(!motor.SetTargetVelocity(0.f), "expired velocity");
  Require(!motor.SetTargetCurrent(0.f), "expired current");
  Require(!motor.SetReset(0, 0.f), "expired reset");
  Require(!motor.SetClearError(0.f), "expired clear");
  Require(!motor.SetZeroOffset(0.f, false, 0.f), "expired zero");
  Require(!motor.SetProfileAccDec(1.f, 1.f), "expired profile acceleration");
  Require(!motor.SetProfileVelocity(1.f), "expired profile velocity");
  Require(!motor.SetCurrentLimit(1.f), "expired current limit");
  Require(!motor.SetMitControl(0.f, 0.f, 1.f, 1.f, 0.f), "expired MIT control");
  Require(!motor.SetMitControlCrc(0.f, 0.f, 1.f, 1.f, 0.f), "expired MIT CRC");
  Require(!motor.SetCollisionThreshold(1.f, 1.f), "expired collision");
  Require(!motor.SetParam(ArmDriver::MotorParam::kTf, 8.f, 0.f), "expired param");
  Require(!motor.GetState(), "expired joint state");
  Require(!motor.GetHighSpeedState(), "expired high speed");
  Require(!motor.GetLowSpeedState(), "expired low speed");
  Require(!motor.GetVersion(0.f), "expired version");
  Require(!motor.GetParam(ArmDriver::MotorParam::kTf, 0.f), "expired get param");
}

bool SelectProtocol() {
  const char* explicit_path = std::getenv("AGX_MOTOR_SDK_ARM_LIB");
  if (explicit_path && *explicit_path) {
    Require(std::filesystem::is_regular_file(explicit_path), "explicit ARM_LIB does not exist");
    std::cout << "protocol: " << explicit_path << '\n';
    return true;
  }
  const char* directory = std::getenv("AGX_MOTOR_SDK_PROTOCOL_DIR");
  const auto path = std::filesystem::path(directory && *directory ? directory :
                                         AGX_SDK_TEST_PROTOCOL_DIR) /
                    "libagx_motor_protocol_arm.so";
  if (!std::filesystem::is_regular_file(path)) {
    std::cout << "SKIP: no protocol library at " << path
              << "; set AGX_MOTOR_SDK_ARM_LIB or AGX_MOTOR_SDK_PROTOCOL_DIR\n";
    return false;
  }
  Require(setenv("AGX_MOTOR_SDK_ARM_LIB", path.c_str(), 1) == 0, "setenv failed");
  std::cout << "protocol: " << path << '\n';
  return true;
}

void CheckProtocolLifecycle() {
  auto& buses = CanBusRegistry::Default();
  auto& motors = MotorRegistry::Default();
  auto stats = std::make_shared<PortStats>();
  auto port = std::make_shared<FakeCanPort>(stats);
  std::weak_ptr<CanPort> weak_port = port;
  auto bus = buses.Acquire("fake0", port);
  Require(buses.Acquire("fake0", port) == bus, "live bus not reused");
  auto driver = bus->driver_shared(Protocol::kArm);
  auto& motor = motors.Acquire("fake0", driver, 1, Config());
  auto other_config = Config();
  other_config.can_channel = "fake1";
  Require(&motors.Acquire("fake1", driver, 1, other_config) != &motor,
          "registry conflated channels");
  std::weak_ptr<MotorDriver> weak_driver = driver;
  buses.Clear();
  Require(bus->can_port() == port.get(), "registry clear invalidated live bus");
  bus.reset();
  Require(stats->closed == 0 && port->IsOpen(), "bus closed external port");
  port.reset();
  Require(!weak_port.expired(), "shared driver failed to retain port");
  Require(driver->SetEnable(1, true), "retained driver not operational");
  Require(stats->sent > 0, "retained driver did not use fake port");
  driver.reset();
  Require(weak_driver.expired(), "motor retained driver strongly");
  Require(stats->closed == 0 && stats->detached > 0, "driver teardown failed to detach only");
  CheckExpired(motor);

  auto next_stats = std::make_shared<PortStats>();
  auto next_port = std::make_shared<FakeCanPort>(next_stats);
  auto next_bus = buses.Acquire("fake0", next_port);
  auto next_config = Config();
  next_config.name = "rebound";
  auto& rebound = motors.Acquire("fake0", next_bus->driver_shared(Protocol::kArm), 1, next_config);
  Require(&rebound == &motor && motor.Name() == "rebound", "registry did not rebind config");
  Require(motor.SetEnable(), "registry did not rebind driver");
  Require(next_stats->sent > 0, "rebound driver not using new port");
  next_bus.reset();
  Require(next_port->IsOpen() && next_stats->closed == 0, "rebound bus closed external port");
  CheckExpired(motor);
  motors.Clear();
  buses.Clear();
  weak_driver.reset();
  Require(weak_port.expired() && stats->destroyed == 1,
          "port not released after driver control block cleanup");
}

CanFrame Frame(uint32_t id) {
  CanFrame frame;
  frame.id = id;
  frame.data = {0x16, 0x37, 0x81, 0x30, 0x12, 0x34, 0x00, 0x00};
  return frame;
}

// Arm (node 1) and RB200 (node 2) share one channel; RX is routed by node ID.
void CheckMixedProtocols() {
  const auto rb200 = std::filesystem::path(std::getenv("AGX_MOTOR_SDK_ARM_LIB")).parent_path() /
                     "libagx_motor_protocol_rb200.so";
  if (!std::filesystem::is_regular_file(rb200)) {
    std::cout << "SKIP mixed protocols: no " << rb200 << '\n';
    return;
  }
  Require(setenv("AGX_MOTOR_SDK_RB200_LIB", rb200.c_str(), 1) == 0, "setenv failed");
  auto& buses = CanBusRegistry::Default();
  auto stats = std::make_shared<PortStats>();
  auto port = std::make_shared<FakeCanPort>(stats);
  auto bus = buses.Acquire("mixed0", port, Protocol::kArm);
  Require(buses.Acquire("mixed0", port, Protocol::kRb200) == bus, "mixed bus not reused");
  auto arm = bus->driver_shared(Protocol::kArm);
  auto rb = bus->driver_shared(Protocol::kRb200);
  Require(arm && rb && arm != rb, "per-protocol drivers missing");
  Require(bus->driver_shared(Protocol::kArm) == arm && bus->driver() == arm.get(), "default driver changed");
  bus->BindNode(1, Protocol::kArm);
  bus->BindNode(2, Protocol::kRb200);
  bus->BindNode(2, Protocol::kRb200);
  bool conflict_rejected = false;
  try {
    bus->BindNode(1, Protocol::kRb200);
  } catch (const std::invalid_argument&) {
    conflict_rejected = true;
  }
  Require(conflict_rejected, "node protocol conflict accepted");

  port->Inject(Frame(0x511));
  port->Inject(Frame(0x512));
  port->Inject(Frame(0x532));
  port->Inject(Frame(0x513));
  Require(arm->GetHighSpeedState(1).has_value(), "arm node 1 feedback not routed");
  Require(!rb->GetHighSpeedState(1).has_value(), "arm node 1 leaked to rb200");
  Require(rb->GetHighSpeedState(2).has_value(), "rb200 node 2 feedback not routed");
  Require(!arm->GetHighSpeedState(2).has_value(), "rb200 node 2 leaked to arm");
  const auto low = std::dynamic_pointer_cast<Rb200Driver>(rb)->GetLowSpeedState(2);
  Require(low && low->status_word == 0x1637 && low->error_code == 0x8130,
          "rb200 low-speed feedback not decoded");
  Require(arm->GetHighSpeedState(3).has_value() && rb->GetHighSpeedState(3).has_value(),
          "unbound node not broadcast");

  const int sent = stats->sent;
  Require(arm->SetEnable(1, true) && rb->SetEnable(2, true), "mixed enable failed");
  Require(stats->sent == sent + 2, "mixed drivers did not share port");
  arm.reset();
  rb.reset();
  bus.reset();
  Require(stats->detached > 0 && stats->closed == 0, "mixed bus teardown");
  buses.Clear();
}

int main(int argc, char** argv) {
  try {
    Require(argc == 2, "expected expired or protocol mode");
    if (std::string(argv[1]) == "expired") {
      auto& registry = MotorRegistry::Default();
      auto& motor = registry.Acquire("fake0", {}, 1, Config());
      CheckExpired(motor);
      auto updated = Config();
      updated.name = "updated";
      Require(&registry.Acquire("fake0", {}, 1, updated) == &motor, "motor not reused");
      Require(motor.Name() == "updated", "config not rebound");
      registry.Clear();
    } else {
      Require(std::string(argv[1]) == "protocol", "unknown mode");
      if (!SelectProtocol()) return 77;
      CheckProtocolLifecycle();
      CheckMixedProtocols();
    }
    std::cout << "lifecycle software checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}