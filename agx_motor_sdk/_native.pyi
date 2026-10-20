"""Type stubs for the C++ pybind11 extension ``agx_motor_sdk._native``.

Public documentation:

* Drivers: :mod:`agx_motor_sdk.motor.arm_driver`
* Per-joint motors: :mod:`agx_motor_sdk.motor.arm_motor_base`
* Mechanism: :mod:`agx_motor_sdk.mechanism.joint_group` (public: `ArmJointGroup` only)
* Feedback types (this file): :class:`DriverStatus`, :class:`HighSpeedFeedback`,
  :class:`LowSpeedFeedback`, :class:`VersionInfo`
"""

from enum import Enum
from typing import Any, List, Optional, Union

# ---------------------------------------------------------------------------
# Motor feedback types (no Python wrapper; docs live here)
# ---------------------------------------------------------------------------

class DriverStatus:
    """Decoded status bits from low-speed feedback ``status_raw`` byte."""

    undervoltage: bool
    motor_overtemp: bool
    driver_overcurrent: bool
    driver_overtemp: bool
    collision_tripped: bool
    driver_error: bool
    enabled: bool
    stall_tripped: bool

class HighSpeedFeedback:
    """Cached high-speed feedback (updated on each matching RX frame)."""

    position: float
    """Joint position [rad]."""
    velocity: float
    """Joint velocity [rad/s]."""
    current: float
    """Phase current [A]."""
    timestamp: float
    timestamp_ns: int

class LowSpeedFeedback:
    """Cached low-speed supplementary feedback (voltage, temperature, status)."""

    bus_voltage_v: float
    driver_temp_deg: float
    motor_temp_deg: float
    bus_current: float
    status_raw: int
    status: DriverStatus
    timestamp: float
    timestamp_ns: int

class VersionInfo:
    """Firmware version query result."""

    software: str
    hardware: str
    motor: str
    timestamp: float
    timestamp_ns: int

# ---------------------------------------------------------------------------
# Arm driver
# ---------------------------------------------------------------------------

class ArmDriver:
    class MotorParam(Enum):
        ID: int
        AC: int
        DC: int
        VV: int
        IQ: int
        OI: int
        OT: int
        TX: int
        TC: int
        TF: int
        SO: int
        PP: int
        KP: int
        KI: int

    def __init__(self, can_port: Any = None) -> None: ...
    def close(self) -> None: ...
    def attach_can_port(self, can_port: Any) -> None: ...
    def handle_rx_once(
        self, can_id: int, data: bytes, timestamp_ns: int = 0
    ) -> bool: ...
    def get_high_speed_feedback(self, node_id: int) -> Optional[HighSpeedFeedback]: ...
    def get_low_speed_feedback(self, node_id: int) -> Optional[LowSpeedFeedback]: ...
    def get_version(self, node_id: int, timeout: float = 1.0) -> Optional[VersionInfo]: ...
    def set_enable(
        self,
        node_id: int,
        enable: bool = True,
        has_brake: bool = False,
        brake_on: bool = False,
    ) -> bool: ...
    def set_target(
        self, node_id: int, value: float, mode: int, timeout: float = 0.0
    ) -> bool: ...
    def set_target_velocity(
        self, node_id: int, velocity: float, timeout: float = 0.0
    ) -> bool: ...
    def set_target_position(
        self, node_id: int, position: float, timeout: float = 0.0
    ) -> bool: ...
    def set_target_current(
        self, node_id: int, current: float, timeout: float = 0.0
    ) -> bool: ...
    def set_reset(self, node_id: int, mode: int = 0, timeout: float = 1.0) -> bool: ...
    def set_clear_error(self, node_id: int, timeout: float = 1.0) -> bool: ...
    def set_zero_offset(
        self,
        node_id: int,
        zero_offset: float = 0.0,
        save_to_flash: bool = False,
        timeout: float = 1.0,
    ) -> bool: ...
    def set_profile_acc_dec(
        self, node_id: int, acceleration: float, deceleration: float
    ) -> bool: ...
    def set_profile_vel(self, node_id: int, velocity: float) -> bool: ...
    def set_current_limit(self, node_id: int, current: float) -> bool: ...
    def set_mit_control(
        self,
        node_id: int,
        p_des: float,
        v_des: float,
        kp: float,
        kd: float,
        t_ff: float,
        t_am: float = 16.0,
    ) -> bool: ...
    def set_mit_control_crc(
        self,
        node_id: int,
        p_des: float,
        v_des: float,
        kp: float,
        kd: float,
        t_ff: float,
        t_am: float = 8.0,
    ) -> bool: ...
    def set_collision_threshold(
        self, node_id: int, current_threshold: float, time_threshold: float
    ) -> bool: ...
    def get_param(
        self, node_id: int, param: MotorParam, timeout: float = 1.0
    ) -> Optional[float]: ...
    def set_param(
        self,
        node_id: int,
        param: MotorParam,
        value: float,
        timeout: float = 1.0,
    ) -> bool: ...

# ---------------------------------------------------------------------------
# CAN (native backend)
# ---------------------------------------------------------------------------

class SocketCanPort:
    """C++ socketcan port exposed to Python."""

    def __init__(self, channel: str = "can0", local_loopback: bool = False) -> None: ...
    def open(self) -> bool: ...
    def close(self) -> None: ...
    def is_open(self) -> bool: ...
    def send(self, can_id: int, data: bytes) -> None: ...
    def set_callback(self, callback: Any) -> None: ...

def wrap_can_port(port: Any) -> Any: ...

# ---------------------------------------------------------------------------
# Mechanism layer (signatures only; docs in agx_motor_sdk.mechanism)
# ---------------------------------------------------------------------------

class JointState:
    position_rad: float
    velocity_rad_s: float
    torque_nm: float

class JointConfig:
    name: str
    node_id: int
    can_channel: str
    polarity: float
    profile_vel_rad_s: Optional[float]
    profile_accel_rad_s2: Optional[float]
    profile_decel_rad_s2: Optional[float]
    mit_tf: Optional[float]
    mit_kp: Optional[float]
    mit_kd: Optional[float]
    torque_k: Optional[float]
    torque_b: Optional[float]
    lower_limit_rad: Optional[float]
    upper_limit_rad: Optional[float]
    has_lower_limit: bool
    has_upper_limit: bool

class GroupConfig:
    can_channel: str
    default_profile_vel_rad_s: Optional[float]
    profile_accel_rad_s2: Optional[float]
    profile_decel_rad_s2: Optional[float]
    mit_tf: Optional[float]
    mit_kp: Optional[float]
    mit_kd: Optional[float]
    torque_k: Optional[float]
    torque_b: Optional[float]
    joints: List[JointConfig]

class ArmMotorBase:
    @property
    def is_valid(self) -> bool: ...
    @property
    def protocol(self) -> str: ...
    @property
    def node_id(self) -> int: ...
    @property
    def can_channel(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def config(self) -> JointConfig: ...
    def set_enable(
        self, enable: bool = True, has_brake: bool = False, brake_on: bool = False
    ) -> bool: ...
    def get_state(self) -> Optional[JointState]: ...
    def set_mit(
        self, position_rad: float, velocity_rad_s: float, torque_nm: float
    ) -> bool: ...
    def configure(self) -> bool: ...
    def get_high_speed_feedback(self) -> Optional[HighSpeedFeedback]: ...
    def get_low_speed_feedback(self) -> Optional[LowSpeedFeedback]: ...
    def get_version(self, timeout: float = 1.0) -> Optional[VersionInfo]: ...
    def set_target(self, value: float, mode: int, timeout: float = 0.0) -> bool: ...
    def set_target_position(self, position: float, timeout: float = 0.0) -> bool: ...
    def set_target_velocity(self, velocity: float, timeout: float = 0.0) -> bool: ...
    def set_target_current(self, current: float, timeout: float = 0.0) -> bool: ...
    def set_reset(self, mode: int = 0, timeout: float = 1.0) -> bool: ...
    def set_clear_error(self, timeout: float = 1.0) -> bool: ...
    def set_zero_offset(
        self,
        zero_offset: float = 0.0,
        save_to_flash: bool = False,
        timeout: float = 1.0,
    ) -> bool: ...
    def set_profile_acc_dec(self, acceleration: float, deceleration: float) -> bool: ...
    def set_profile_vel(self, velocity: float) -> bool: ...
    def set_current_limit(self, current: float) -> bool: ...
    def set_mit_control(
        self,
        p_des: float,
        v_des: float,
        kp: float,
        kd: float,
        t_ff: float,
        t_am: float = 16.0,
    ) -> bool: ...
    def set_mit_control_crc(
        self,
        p_des: float,
        v_des: float,
        kp: float,
        kd: float,
        t_ff: float,
        t_am: float = 8.0,
    ) -> bool: ...
    def set_collision_threshold(
        self, current_threshold: float, time_threshold: float
    ) -> bool: ...
    def get_param(
        self, param: ArmDriver.MotorParam, timeout: float = 1.0
    ) -> Optional[float]: ...
    def set_param(
        self, param: ArmDriver.MotorParam, value: float, timeout: float = 1.0
    ) -> bool: ...

class JointGroupBase:
    INVALID_JOINT_INDEX: int

class ArmJointGroup(JointGroupBase):
    def __init__(self, config: GroupConfig) -> None: ...
    def __init__(self, config: GroupConfig, can_channel: str) -> None: ...
    def connect(self) -> bool: ...
    def close(self) -> None: ...
    def is_connected(self) -> bool: ...
    def enable(self) -> bool: ...
    def disable(self) -> bool: ...
    def configure(self) -> bool: ...
    @property
    def joint_count(self) -> int: ...
    @property
    def joint_names(self) -> List[str]: ...
    @property
    def protocol(self) -> str: ...
    @property
    def can_channel(self) -> str: ...
    @property
    def can_channels(self) -> List[str]: ...
    @property
    def config(self) -> GroupConfig: ...
    def index_of(self, joint_name: str) -> int: ...
    def is_valid_joint_index(self, index: int) -> bool: ...
    def node_id(self, index: int) -> int: ...
    def joint_can_channel(self, index: int) -> str: ...
    def joint_config(self, index: int) -> JointConfig: ...
    def set_target_position(
        self, index: int, position_rad: float, timeout: float = 0.0
    ) -> bool: ...
    def get_state(self, index: int) -> Optional[JointState]: ...
    def motor(self, index: int) -> ArmMotorBase: ...
    def attach_can_port(self, can_port: Any, channel: Optional[str] = None) -> None: ...
    def set_mit(
        self,
        index: int,
        position_rad: float,
        velocity_rad_s: float,
        torque_nm: float,
    ) -> bool: ...
    @staticmethod
    def from_yaml(
        yaml_path: str,
        module_key: str,
        can_channel: Optional[str] = None,
    ) -> ArmJointGroup: ...

def create_joint_group(config: GroupConfig) -> JointGroupBase: ...
def create_joint_group(config: GroupConfig, can_channel: str) -> JointGroupBase: ...

def load_module_config(yaml_path: str, module_key: str) -> GroupConfig: ...
