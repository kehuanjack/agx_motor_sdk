#!/usr/bin/env python
import os
import runpy
import subprocess
import sys
from pathlib import Path

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext

_here = Path(__file__).resolve().parent
if str(_here) not in sys.path:
    sys.path.insert(0, str(_here))

from protocol_fetch import get_cmdclass  # noqa: E402


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        super().__init__(name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            subprocess.check_output(["cmake", "--version"])
        except OSError as exc:
            raise RuntimeError("CMake is required to build agx_motor_sdk._native") from exc
        for ext in self.extensions:
            self.build_extension(ext)
        super().run()

    def build_extension(self, ext):
        ext_fullpath = Path.cwd() / self.get_ext_fullpath(ext.name)
        extdir = ext_fullpath.parent.resolve()
        cfg = "Debug" if self.debug else "Release"
        cmake_args = [
            f"-DCMAKE_BUILD_TYPE={cfg}",
            f"-DPYTHON_BINDING=ON",
            f"-DAGX_MOTOR_SDK_PIP_BUILD=ON",
            f"-DPYBIND_OUTPUT_LIBDIR={extdir}{os.sep}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
        ]
        build_temp = Path(self.build_temp) / ext.name
        build_temp.mkdir(parents=True, exist_ok=True)
        subprocess.run(["cmake", ext.sourcedir, *cmake_args], cwd=build_temp, check=True)
        subprocess.run(["cmake", "--build", ".", "--", "-j2"], cwd=build_temp, check=True)


cmdclass = get_cmdclass()
cmdclass["build_ext"] = CMakeBuild
version_info = runpy.run_path(str(_here / "agx_motor_sdk" / "version.py"))

setup(
    name="agx_motor_sdk",
    version=version_info["__version__"],
    description="AgileX motor driver SDK (arm protocol over CAN)",
    long_description=(_here / "README.md").read_text(encoding="utf-8"),
    long_description_content_type="text/markdown",
    author="kehuanjack, Agilex Robotics Co., Ltd.",
    license="Apache-2.0",
    python_requires=">=3.8",
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: POSIX :: Linux",
    ],
    project_urls={"Homepage": "https://github.com/kehuanjack/agx_motor_sdk"},
    packages=find_packages(include=["agx_motor_sdk", "agx_motor_sdk.*"]),
    include_package_data=True,
    package_data={
        "agx_motor_sdk": [
            "py.typed",
            "*.pyi",
            "protocol/*.so",
            "protocol/*.dll",
            "protocol/*.dylib",
        ],
    },
    install_requires=[],
    extras_require={
        "python-can": ["python-can>=3.3.4"],
        "dev": ["pytest>=6.2.5,<8", "python-can>=3.3.4"],
    },
    ext_modules=[CMakeExtension("agx_motor_sdk._native", sourcedir=str(_here))],
    cmdclass=cmdclass,
    zip_safe=False,
)
