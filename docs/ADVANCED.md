# Advanced usage

For the standard installation and first connection, start with [README](../README.md).
Use this page only when changing installation or transport defaults.

## Offline installation

Obtain a protocol library matching your CPU architecture and place it at
`agx_motor_sdk/protocol/libagx_motor_protocol_arm.so` under the repository root.
Then choose your installation method:

```bash
AGX_MOTOR_SDK_SKIP_DOWNLOAD=1 uv pip install ".[python-can]"
```

```bash
cmake -S . -B build -DAGX_MOTOR_SDK_FETCH_PROTOCOL=OFF -DPYTHON_BINDING=OFF
cmake --build build -j4
```

Offline mode uses the supplied library without release verification. Verify its
source and integrity yourself. In normal online mode, missing release metadata
or a missing/invalid SHA256 stops installation, even if a local library exists.
Failed downloads or verification leave the existing library unchanged.

## Custom protocol library

Installed libraries are normally found automatically. For a custom location:

```bash
export AGX_MOTOR_SDK_ARM_LIB=/absolute/path/to/libagx_motor_protocol_arm.so
```

A library in the current working directory's `protocol/` folder is not searched
automatically. Ensure the library matches your CPU architecture.

## Python-can backend

The default `cpp` backend receives feedback automatically. To use `python-can`,
install the optional dependency from the repository root:

```bash
uv pip install ".[python-can]"
```

This backend requires a receive loop while driver commands run:

```python
import threading
from agx_motor_sdk import ArmDriver, create_can_port, create_can_port_config

config = create_can_port_config(channel="can0", backend="python-can", timeout=0.1)
bus = create_can_port(config)
driver = ArmDriver(bus)
stop = threading.Event()

def receive():
    while not stop.is_set():
        bus.recv()

receiver = threading.Thread(target=receive)
receiver.start()
try:
    print(driver.get_version(6, timeout=0.5))
finally:
    stop.set()
    receiver.join()
    driver.close()
    bus.close()
```

Keep the receive loop running until commands have finished. Avoid issuing blocking
driver queries from receive callbacks; the callback must return to receive replies.

## Safe shutdown

- Stop issuing commands before closing groups, drivers or ports. Do not run
  connection, closure, port replacement or group changes concurrently with commands.
- Closing a group or driver does not disable motors. Handle motor disable and
  emergency stopping separately, according to your mechanism's safety requirements.
- If you supplied an external port, close it yourself after all its users have
  stopped. Groups and Python drivers keep supplied ports alive until replacement
  or destruction, including after `close()`.
- With C++ `ArmDriver(CanPort*)`, keep the port alive until the driver is closed
  or destroyed.
- Keep callbacks short. Removing or replacing a SocketCAN callback may wait for
  an active callback to finish; callbacks may unregister themselves.
- Do not destroy a port or driver inside its receive callback. If a callback
  calls `close()`, call it again outside the callback before destruction or reopening.
- Clear callbacks and close ports before Python interpreter shutdown.

## Environment variables

These are optional; the standard installation does not need overrides.

| Variable | Purpose |
|----------|---------|
| `AGX_MOTOR_SDK_ARM_LIB` | Absolute path to a custom protocol library |
| `AGX_MOTOR_SDK_CAN_BACKEND` | `cpp` (default) or `python-can` |
| `AGX_MOTOR_SDK_SKIP_DOWNLOAD` | Set to `1` for explicit offline installation |
| `AGX_MOTOR_SDK_PROTOCOL_VERSION` | Protocol version used in asset name and default tag |
| `AGX_MOTOR_SDK_PROTOCOL_TAG` | Override release tag only, not the asset version |
| `AGX_MOTOR_SDK_GITHUB_REPO` | Binary release repository; default `kehuanjack/agx_motor_protocol_releases` |
| `AGX_MOTOR_SDK_GITHUB_TOKEN` | Release read-access token; falls back to `GH_TOKEN`, then `GITHUB_TOKEN` |

The default protocol version is selected by the SDK independently of its own
version. Releases use tag `arm-can-v{version}` and asset
`agx-arm-can-v{version}-linux-{arch}.so`. Configure tokens in your environment or
CI secret store, never in source files. Private releases require repository read access.

`AGX_MOTOR_SDK_FETCH_PROTOCOL` is a **CMake option**, not an environment variable.
Pass `-DAGX_MOTOR_SDK_FETCH_PROTOCOL=OFF` to disable fetching; default is `ON`.