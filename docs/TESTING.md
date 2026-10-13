# Software checks

Run these commands from the SDK repository root. These checks need no CAN hardware
and do not validate device replies, ACKs or physical motor operation.
For installation and hardware connection, see [README](../README.md).

## C++ checks

```bash
cmake -S . -B /tmp/agx-sdk-software-build \
    -DAGX_MOTOR_SDK_FETCH_PROTOCOL=OFF -DPYTHON_BINDING=OFF -DBUILD_TESTING=ON
cmake --build /tmp/agx-sdk-software-build -j4
ctest --test-dir /tmp/agx-sdk-software-build -L software --output-on-failure
```

Coverage includes configuration validation, resource cleanup, callback handling
and use of the installed C++ package. No CAN interface is opened.

The protocol lifecycle check needs a local library. Set `AGX_MOTOR_SDK_ARM_LIB`
to an absolute file path, or `AGX_MOTOR_SDK_PROTOCOL_DIR` to its directory.
Without an override it uses the build's `protocol/` directory. A missing directory
library skips that check; an invalid explicit file or unloadable library fails.

## Python checks

Use a uv environment with pytest installed:

```bash
uv venv --python 3.11
uv pip install "pytest>=6.2.5,<8"
```

Downloader checks do not require a protocol library or network access:

```bash
env -u PYTHONPATH -u PYTHONHOME PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    .venv/bin/python -m pytest tests/test_protocol_fetch.py
```

Do not add `-I` to this command: it needs to import the downloader from the source
tree. Release access is mocked.

Lifecycle checks require a built Python binding and a local protocol library.
Replace the two paths below with your build outputs:

```bash
AGX_SDK_TEST_NATIVE_DIR=/absolute/path/to/build/lib \
AGX_MOTOR_SDK_ARM_LIB=/absolute/path/to/libagx_motor_protocol_arm.so \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
.venv/bin/python -I -m pytest tests/test_native_lifecycle.py
```

Passing these checks does not replace testing against your actual CAN adapter,
motor firmware and mechanism. A virtual CAN test can verify software transport
but cannot prove hardware timing or motor behavior.