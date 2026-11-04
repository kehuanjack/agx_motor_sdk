"""Multi-joint module controller for one YAML block.

The public mechanism API is::

    JointGroup.from_yaml(...)
        → connect() → motor(index) → per-joint commands

Group objects manage shared CAN backends and module-level batch operations
(``enable``, ``configure``). Per-joint control goes through :meth:`JointGroup.motor`,
or group-level shortcuts :meth:`JointGroup.set_target_position` /
:meth:`JointGroup.get_state`.

YAML schema (full tables): ``YAML_CONFIG.md`` in the package root.

Configuration types
-------------------

Parsed by :func:`~agx_motor_sdk.mechanism.load_module_config` and exposed on each group as
:attr:`JointGroup.config`:

* :class:`~agx_motor_sdk.mechanism.GroupConfig` — one module (``can_channel``,
  profile/MIT defaults, ``joints`` list).
* :class:`~agx_motor_sdk.mechanism.JointConfig` — per joint: ``name``, ``node_id`` (1–15),
  ``polarity`` (default ``1.0``; ``0`` = pass-through), optional ``can_channel``
  override, profile/MIT/torque/limits.
* :class:`~agx_motor_sdk.mechanism.JointState` — feedback from ``motor.get_state()``:
  ``position_rad`` [rad], ``velocity_rad_s`` [rad/s], ``torque_nm`` [N·m].

Joint index
-----------

* Valid joint index: ``0 <= index < joint_count``.
* :attr:`INVALID_JOINT_INDEX` is ``-1`` (unknown name from :meth:`index_of`).
* :meth:`motor` / :meth:`get_state` / :meth:`set_target_position`: invalid index → no-op /
  ``None`` / ``False`` (no exception).
* :meth:`node_id` / :meth:`joint_config`: invalid index → ``IndexError``.

Polarity (YAML ``polarity`` per joint)
--------------------------------------

Linear coefficient between joint space and motor space (default ``1.0`` if omitted):

Polarity is applied inside per-joint motor methods (``set_target_position``,
``get_high_speed_feedback``, ``set_mit_control``, etc.). ``get_state`` and ``set_mit``
compose those helpers and do not transform again.

* Command: ``motor_value = joint_value / polarity`` (``polarity == 0`` → pass-through)
* Feedback: ``joint_value = motor_value × polarity`` for position, velocity, current, and torque
* ``±1``: same magnitude; negative reverses direction

Per-joint motor API
-------------------

See :mod:`agx_motor_sdk.motor.motor_base` / :mod:`agx_motor_sdk.motor.arm_motor` / :mod:`agx_motor_sdk.motor.rb200_motor` for full method docs and parameter ranges.

Quick start::

    from agx_motor_sdk.mechanism import JointGroup

    head = JointGroup.from_yaml("robot_motors.yaml", "head")
    head.connect()
    head.configure()
    head.enable()

    idx = head.index_of("head_pitch_joint")
    if idx != JointGroup.INVALID_JOINT_INDEX:
        head.motor(idx).set_target_position(0.5)
        state = head.get_state(idx)
"""

from typing import Any, ClassVar, List, Optional

from agx_motor_sdk import _native
from agx_motor_sdk._native import GroupConfig
from agx_motor_sdk._native import JointConfig
from agx_motor_sdk._native import JointGroup as _JointGroup
from agx_motor_sdk._native import JointState
from agx_motor_sdk.motor.arm_motor import ArmMotor
from agx_motor_sdk.motor.motor_base import MotorBase
from agx_motor_sdk.motor.rb200_motor import Rb200Motor


class JointGroup:
    """YAML module controller (arm and RB200 joints, 1 Mbps CAN).

    ``motor(index)`` returns :class:`~agx_motor_sdk.motor.ArmMotor` or
    :class:`~agx_motor_sdk.motor.Rb200Motor` according to the joint ``protocol``.
    Joint-space units are rad and rad/s. One module may mix both protocols on
    the same CAN channel.

    Example::

        group = JointGroup.from_yaml("robot_motors.yaml", "head")
        group.connect()
        group.motor(0).set_mit(0.0, 0.0, 0.0)
    """

    INVALID_JOINT_INDEX: ClassVar[int] = _JointGroup.INVALID_JOINT_INDEX
    """Sentinel returned by :meth:`index_of` when a joint name is unknown (always -1)."""

    def __init__(self, config: GroupConfig, can_channel: Optional[str] = None) -> None:
        """Create a joint group from parsed config.

        Args:
            config: Parsed module configuration from :func:`load_module_config`.
            can_channel: Optional module default CAN channel override.
        """
        if can_channel is not None:
            self._native = _JointGroup(config, can_channel)
        else:
            self._native = _JointGroup(config)

    @classmethod
    def from_yaml(
        cls,
        yaml_path: str,
        module_key: str,
        can_channel: Optional[str] = None,
    ) -> "JointGroup":
        """Load a module from YAML.

        Args:
            yaml_path: Path to robot motors YAML.
            module_key: Module key in the YAML file.
            can_channel: Optional CAN channel override.

        Raises:
            RuntimeError: On YAML load/parse errors.
        """
        obj = cls.__new__(cls)
        obj._native = _JointGroup.from_yaml(yaml_path, module_key, can_channel)
        return obj

    def connect(self) -> bool:
        """Acquire and open all CAN backends required by this module."""
        return self._native.connect()

    def close(self) -> None:
        """Release this group's references to shared CAN backends."""
        self._native.close()

    def is_connected(self) -> bool:
        """Return whether :meth:`connect` succeeded and :meth:`close` has not been called."""
        return self._native.is_connected()

    def enable(self, interval: float = 0.0) -> bool:
        """Enable all joints in this module.

        Args:
            interval: Delay in seconds between consecutive joint commands.
                Use a small value if a board drops back-to-back frames.

        Raises:
            ValueError: If ``interval`` is negative or not finite.
        """
        return self._native.enable(interval)

    def disable(self, interval: float = 0.0) -> bool:
        """Disable all joints in this module.

        Args:
            interval: Delay in seconds between consecutive joint commands.

        Raises:
            ValueError: If ``interval`` is negative or not finite.
        """
        return self._native.disable(interval)

    def configure(self) -> bool:
        """Apply YAML profile / MIT settings to all joints."""
        return self._native.configure()

    @property
    def joint_count(self) -> int:
        """Number of joints in this module."""
        return self._native.joint_count

    @property
    def joint_names(self) -> List[str]:
        """Joint names in YAML list order."""
        return self._native.joint_names

    @property
    def protocol(self) -> str:
        """Module default protocol: ``"arm"`` or ``"rb200"``."""
        return self._native.protocol

    @property
    def can_channel(self) -> str:
        """Module default CAN channel from YAML ``can_channel``."""
        return self._native.can_channel

    @property
    def can_channels(self) -> List[str]:
        """Distinct CAN channels used by joints."""
        return self._native.can_channels

    @property
    def config(self) -> GroupConfig:
        """Parsed :class:`~agx_motor_sdk.mechanism.GroupConfig` backing this group."""
        return self._native.config

    def index_of(self, joint_name: str) -> int:
        """Look up a joint index by YAML ``joint_name``."""
        return int(self._native.index_of(joint_name))

    def is_valid_joint_index(self, index: int) -> bool:
        """Return whether ``index`` is in ``[0, joint_count)``."""
        return bool(self._native.is_valid_joint_index(index))

    def node_id(self, index: int) -> int:
        """Return the CAN node ID (1–15) for a joint index."""
        return int(self._native.node_id(index))

    def joint_can_channel(self, index: int) -> str:
        """Return the resolved CAN channel for a joint index."""
        return self._native.joint_can_channel(index)

    def joint_config(self, index: int) -> JointConfig:
        """Return static :class:`~agx_motor_sdk.mechanism.JointConfig` for a joint."""
        return self._native.joint_config(index)

    def set_target_position(
        self, index: int, position_rad: float, timeout: float = 0.0
    ) -> bool:
        """Joint-space position command on one joint [rad] (limits + polarity)."""
        return bool(self._native.set_target_position(index, position_rad, timeout))

    def get_state(self, index: int) -> Optional[JointState]:
        """Read joint-space feedback for one joint."""
        return self._native.get_state(index)

    def attach_can_port(self, can_port: Any, channel: Optional[str] = None) -> None:
        """Inject an external CAN port before :meth:`connect`."""
        self._native.attach_can_port(can_port, channel)

    def motor(self, index: int) -> MotorBase:
        """Return the per-joint motor for ``index``.

        Returns:
            :class:`~agx_motor_sdk.motor.ArmMotor` for arm joints,
            :class:`~agx_motor_sdk.motor.Rb200Motor` for RB200 joints, or a no-op
            :class:`~agx_motor_sdk.motor.MotorBase` (``is_valid=False``) for an
            invalid index.
        """
        native = self._native.motor(index)
        if isinstance(native, _native.ArmMotor):
            return ArmMotor(native)
        if isinstance(native, _native.Rb200Motor):
            return Rb200Motor(native)
        return MotorBase(native)

    def set_mit(
        self,
        index: int,
        position_rad: float,
        velocity_rad_s: float,
        torque_nm: float,
    ) -> bool:
        """Joint-space MIT command on one arm joint (``False`` for RB200 joints)."""
        return bool(
            self._native.set_mit(index, position_rad, velocity_rad_s, torque_nm)
        )


__all__ = [
    "JointGroup",
]
