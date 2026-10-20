import hashlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request
from unittest.mock import patch

import protocol_fetch


class ProtocolFetchTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.library = self.root / "agx_motor_sdk/protocol/libagx_motor_protocol_arm.so"
        self.payload = b"verified protocol binary"
        self.asset = "agx-arm-can-v1.2.3-linux-x86_64.so"
        self.environment = patch.dict(os.environ, {}, clear=True)
        self.environment.start()
        self.addCleanup(self.environment.stop)
        for target, value in (
            ("_ROOT", self.root),
            ("_read_protocol_version", "1.2.3"),
            ("_host_platform_arch", ("linux", "x86_64")),
        ):
            replacement = patch.object(protocol_fetch, target, value) if target == "_ROOT" else patch.object(protocol_fetch, target, return_value=value)
            replacement.start()
            self.addCleanup(replacement.stop)
        platform_patch = patch.object(protocol_fetch.sys, "platform", "linux")
        platform_patch.start()
        self.addCleanup(platform_patch.stop)

    def metadata(self, digest=None):
        digest = hashlib.sha256(self.payload).hexdigest() if digest is None else digest
        return {
            "body": "<!-- AGX_MOTOR_SDK_PROTOCOL_SHA256 -->\n```json\n"
            + json.dumps({self.asset: digest}) + "\n```",
            "assets": [{"name": self.asset, "url": "https://api.github.com/repos/kehuanjack/agx_motor_protocol_releases/releases/assets/123"}],
        }

    def network(self, metadata=None, payload=None):
        return patch.object(protocol_fetch, "urlopen", side_effect=[
            io.BytesIO(json.dumps(self.metadata() if metadata is None else metadata).encode()),
            io.BytesIO(self.payload if payload is None else payload),
        ])

    def test_default_repo_tag_asset_and_public_url(self):
        with self.network() as network:
            protocol_fetch.download_protocol_libs_if_needed()
        metadata_request = network.call_args_list[0].args[0]
        self.assertEqual(metadata_request.full_url, "https://api.github.com/repos/kehuanjack/agx_motor_protocol_releases/releases/tags/arm-can-v1.2.3")
        asset_request = network.call_args_list[1].args[0]
        asset_url = asset_request if isinstance(asset_request, str) else asset_request.full_url
        self.assertEqual(asset_url, "https://github.com/kehuanjack/agx_motor_protocol_releases/releases/download/arm-can-v1.2.3/" + self.asset)
        self.assertEqual(self.library.read_bytes(), self.payload)

    def test_bad_hash_preserves_existing_library(self):
        self.library.parent.mkdir(parents=True)
        self.library.write_bytes(b"existing library")
        with self.network(payload=b"corrupt"), self.assertRaisesRegex(RuntimeError, "SHA256 mismatch"):
            protocol_fetch.download_protocol_libs_if_needed()
        self.assertEqual(self.library.read_bytes(), b"existing library")

    def test_repo_and_tag_overrides(self):
        os.environ.update(AGX_MOTOR_SDK_GITHUB_REPO="example/releases", AGX_MOTOR_SDK_PROTOCOL_TAG="custom/tag")
        with self.network() as network:
            protocol_fetch.download_protocol_libs_if_needed()
        self.assertEqual(network.call_args_list[0].args[0].full_url, "https://api.github.com/repos/example/releases/releases/tags/custom%2Ftag")
        self.assertEqual(network.call_args_list[1].args[0], "https://github.com/example/releases/releases/download/custom%2Ftag/" + self.asset)

    def test_platform_asset_names_and_destinations(self):
        for plat, system, extension in (("linux", "linux", "so"), ("windows", "win32", "dll"), ("darwin", "darwin", "dylib")):
            with self.subTest(platform=plat), patch.object(protocol_fetch.sys, "platform", system):
                self.assertEqual(protocol_fetch._release_asset_name("arm", plat, "aarch64", "2.0.0"), f"agx-arm-can-v2.0.0-{plat}-aarch64.{extension}")
                self.assertEqual(protocol_fetch._dest_filename("arm"), f"libagx_motor_protocol_arm.{extension}")

    def test_authenticated_metadata_and_asset_headers(self):
        os.environ["AGX_MOTOR_SDK_GITHUB_TOKEN"] = "test-token"
        with patch.object(protocol_fetch, "build_opener") as factory, patch.object(protocol_fetch, "urlopen") as public, patch("builtins.print") as output:
            opener = factory.return_value
            opener.open.side_effect = [io.BytesIO(json.dumps(self.metadata()).encode()), io.BytesIO(self.payload)]
            protocol_fetch.download_protocol_libs_if_needed()
        public.assert_not_called()
        self.assertIsInstance(factory.call_args.args[0], protocol_fetch._NoAuthRedirectHandler)
        requests = [call.args[0] for call in opener.open.call_args_list]
        self.assertEqual(requests[0].full_url, "https://api.github.com/repos/kehuanjack/agx_motor_protocol_releases/releases/tags/arm-can-v1.2.3")
        self.assertEqual(requests[1].full_url, self.metadata()["assets"][0]["url"])
        for request, accept in zip(requests, ("application/vnd.github+json", "application/octet-stream")):
            self.assertEqual(request.get_header("Authorization"), "Bearer test-token")
            self.assertNotIn("Authorization", request.headers)
            self.assertEqual(request.get_header("Accept"), accept)
            self.assertEqual(request.get_header("User-agent"), "agx_motor_sdk-setup")
        self.assertNotIn("test-token", str(output.call_args_list))
        self.assertEqual(self.library.read_bytes(), self.payload)

    def test_token_priority_and_fallbacks(self):
        for name in ("GITHUB_TOKEN", "GH_TOKEN", "AGX_MOTOR_SDK_GITHUB_TOKEN"):
            os.environ[name] = name
            self.assertEqual(protocol_fetch._github_token(), name)

    def test_authenticated_metadata_errors_fail_without_public_fallback(self):
        os.environ["GH_TOKEN"] = "test-token"
        failures = [HTTPError("https://api.github.com", code, "test-token", {}, None) for code in (401, 403, 404)]
        failures.append(URLError("test-token"))
        for failure in failures:
            with self.subTest(failure=type(failure).__name__), patch.object(protocol_fetch, "build_opener") as factory, patch.object(protocol_fetch, "urlopen") as public:
                factory.return_value.open.side_effect = failure
                with self.assertRaisesRegex(RuntimeError, "authenticated GitHub release metadata request failed") as error:
                    protocol_fetch.download_protocol_libs_if_needed()
                self.assertNotIn("test-token", str(error.exception))
                public.assert_not_called()
                self.assertFalse(self.library.exists())

    def test_authenticated_malformed_metadata_fails(self):
        os.environ["GITHUB_TOKEN"] = "test-token"
        for content in (b"not json", b"[]"):
            with self.subTest(content=content), patch.object(protocol_fetch, "build_opener") as factory:
                factory.return_value.open.return_value = io.BytesIO(content)
                with self.assertRaisesRegex(RuntimeError, "authenticated GitHub release metadata"):
                    protocol_fetch.download_protocol_libs_if_needed()

    def test_authenticated_bad_hash_preserves_existing_library(self):
        os.environ["GH_TOKEN"] = "test-token"
        self.library.parent.mkdir(parents=True)
        self.library.write_bytes(b"existing library")
        with patch.object(protocol_fetch, "build_opener") as factory:
            factory.return_value.open.side_effect = [io.BytesIO(json.dumps(self.metadata()).encode()), io.BytesIO(b"corrupt")]
            with self.assertRaisesRegex(RuntimeError, "SHA256 mismatch"):
                protocol_fetch.download_protocol_libs_if_needed()
        self.assertEqual(self.library.read_bytes(), b"existing library")

    def test_authenticated_asset_failure_preserves_existing_library(self):
        os.environ["GH_TOKEN"] = "test-token"
        self.library.parent.mkdir(parents=True)
        self.library.write_bytes(b"existing library")
        with patch.object(protocol_fetch, "build_opener") as factory, patch.object(protocol_fetch, "urlopen") as public:
            factory.return_value.open.side_effect = [io.BytesIO(json.dumps(self.metadata()).encode()), URLError("test-token")]
            with self.assertRaisesRegex(RuntimeError, "Authenticated GitHub asset download failed") as error:
                protocol_fetch.download_protocol_libs_if_needed()
            self.assertNotIn("test-token", str(error.exception))
            public.assert_not_called()
        self.assertEqual(self.library.read_bytes(), b"existing library")

    def test_untrusted_asset_urls_rejected_before_download(self):
        os.environ["GH_TOKEN"] = "test-token"
        for url in ("https://example.com/asset", "https://api.github.com.evil/asset", "http://api.github.com/asset", "https://api.github.com/repos/other/repo/releases/assets/123"):
            metadata = self.metadata()
            metadata["assets"][0]["url"] = url
            with self.subTest(url=url), patch.object(protocol_fetch, "build_opener") as factory:
                factory.return_value.open.return_value = io.BytesIO(json.dumps(metadata).encode())
                with self.assertRaisesRegex(RuntimeError, "untrusted"):
                    protocol_fetch.download_protocol_libs_if_needed()
                self.assertEqual(factory.return_value.open.call_count, 1)

    def test_redirect_drops_authorization(self):
        request = Request("https://api.github.com/repos/example/releases/releases/assets/123")
        request.add_header("Authorization", "Bearer test-token")
        request.add_unredirected_header("Authorization", "Bearer test-token")
        redirected = protocol_fetch._NoAuthRedirectHandler().redirect_request(request, None, 302, "Found", {}, "https://example.com/asset")
        self.assertIsNotNone(redirected)
        self.assertIsNone(redirected.get_header("Authorization"))

    def test_matching_hash_uses_cache(self):
        self.library.parent.mkdir(parents=True)
        self.library.write_bytes(self.payload)
        with self.network() as network:
            protocol_fetch.download_protocol_libs_if_needed()
        self.assertEqual(network.call_count, 1)
        self.assertEqual(self.library.read_bytes(), self.payload)

    def test_mismatching_cache_replaced_after_verified_download(self):
        self.library.parent.mkdir(parents=True)
        self.library.write_bytes(b"old binary")
        with self.network():
            protocol_fetch.download_protocol_libs_if_needed()
        self.assertEqual(self.library.read_bytes(), self.payload)

    def test_public_metadata_failure_rejects_download(self):
        with patch.object(protocol_fetch, "urlopen", side_effect=[URLError("offline metadata"), io.BytesIO(self.payload)]) as network:
            with self.assertRaisesRegex(RuntimeError, "GitHub release metadata request failed"):
                protocol_fetch.download_protocol_libs_if_needed()
        self.assertEqual(network.call_count, 1)
        self.assertFalse(self.library.exists())

    def test_public_malformed_metadata_fails(self):
        for content in (b"not json", b"[]"):
            with self.subTest(content=content), patch.object(protocol_fetch, "urlopen", return_value=io.BytesIO(content)) as network:
                with self.assertRaisesRegex(RuntimeError, "GitHub release metadata request failed"):
                    protocol_fetch.download_protocol_libs_if_needed()
                self.assertEqual(network.call_count, 1)
                self.assertFalse(self.library.exists())

    def test_local_without_metadata_is_preserved(self):
        self.library.parent.mkdir(parents=True)
        self.library.write_bytes(b"local binary")
        with patch.object(protocol_fetch, "urlopen", side_effect=URLError("offline")) as network:
            with self.assertRaisesRegex(RuntimeError, "GitHub release metadata request failed"):
                protocol_fetch.download_protocol_libs_if_needed()
        self.assertEqual(network.call_count, 1)
        self.assertEqual(self.library.read_bytes(), b"local binary")

    def test_missing_or_invalid_digest_rejected_before_asset_download(self):
        self.library.parent.mkdir(parents=True)
        self.library.write_bytes(b"existing library")
        for metadata in ({}, {"body": "no digest"}, *(self.metadata(digest) for digest in ("", "a" * 63, "a" * 65, "g" * 64, " " + "a" * 64, 123, [], {}))):
            with self.subTest(metadata=metadata), self.network(metadata=metadata) as network:
                with self.assertRaisesRegex(RuntimeError, "Missing or invalid SHA256"):
                    protocol_fetch.download_protocol_libs_if_needed()
                self.assertEqual(network.call_count, 1)
                self.assertEqual(self.library.read_bytes(), b"existing library")

    def test_uppercase_digest_is_valid(self):
        with self.network(metadata=self.metadata(hashlib.sha256(self.payload).hexdigest().upper())):
            protocol_fetch.download_protocol_libs_if_needed()
        self.assertEqual(self.library.read_bytes(), self.payload)

    def test_atomic_failure_preserves_existing_library_and_cleans_temporary(self):
        self.library.parent.mkdir(parents=True)
        self.library.write_bytes(b"existing library")
        for operation in ("fsync", "replace"):
            with self.subTest(operation=operation), self.network(), patch.object(protocol_fetch.os, operation, side_effect=OSError("disk failure")):
                with self.assertRaisesRegex(OSError, "disk failure"):
                    protocol_fetch.download_protocol_libs_if_needed()
                self.assertEqual(self.library.read_bytes(), b"existing library")
                self.assertEqual(list(self.library.parent.iterdir()), [self.library])

    def test_atomic_replace_uses_verified_same_directory_temporary(self):
        self.library.parent.mkdir(parents=True)
        self.library.write_bytes(b"existing library")
        original_replace = os.replace

        def replace(source, destination):
            self.assertEqual(source.parent, self.library.parent)
            self.assertEqual(source.read_bytes(), self.payload)
            self.assertEqual(destination, self.library)
            self.assertEqual(self.library.read_bytes(), b"existing library")
            original_replace(source, destination)

        with self.network(), patch.object(protocol_fetch.os, "replace", side_effect=replace) as replacement:
            protocol_fetch.download_protocol_libs_if_needed()
        replacement.assert_called_once()
        self.assertEqual(self.library.read_bytes(), self.payload)
        self.assertEqual(list(self.library.parent.iterdir()), [self.library])

    def test_bad_hash_does_not_create_library(self):
        with self.network(payload=b"corrupt"), self.assertRaisesRegex(RuntimeError, "SHA256 mismatch"):
            protocol_fetch.download_protocol_libs_if_needed()
        self.assertFalse(self.library.exists())

    def test_skip_download_avoids_network(self):
        os.environ["AGX_MOTOR_SDK_SKIP_DOWNLOAD"] = "1"
        with patch.object(protocol_fetch, "urlopen") as network:
            protocol_fetch.download_protocol_libs_if_needed()
        network.assert_not_called()


@unittest.skipUnless(shutil.which("cmake"), "cmake is unavailable")
class CMakeProtocolFetchTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.cmake_dir = Path(__file__).resolve().parents[1] / "cmake"
        self.library = self.root / "agx_motor_sdk/protocol/libagx_motor_protocol_arm.so"
        self.library.parent.mkdir(parents=True)
        self.library.write_bytes(b"unverified old library")
        self.staged = self.root / "build/protocol/libagx_motor_protocol_arm.so"
        (self.root / "protocol_fetch.py").write_text("raise SystemExit(7)\n")
        (self.root / "FindPython3.cmake").write_text(f'set(Python3_EXECUTABLE "{sys.executable}")\n')
        (self.root / "CMakeLists.txt").write_text(
            'cmake_minimum_required(VERSION 3.14)\nproject(fetch_probe NONE)\n'
            'list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")\n'
            f'include("{self.cmake_dir / "fetch_protocol_libs.cmake"}")\n'
            'agx_motor_sdk_fetch_protocol_libs()\nagx_motor_sdk_stage_protocol_libs()\n'
        )

    def configure(self, fetch, skip=False):
        environment = os.environ.copy()
        for name in ("PYTHONPATH", "PYTHONHOME", "AGX_MOTOR_SDK_SKIP_DOWNLOAD"):
            environment.pop(name, None)
        if skip:
            environment["AGX_MOTOR_SDK_SKIP_DOWNLOAD"] = "1"
        return subprocess.run(
            [shutil.which("cmake"), "-S", str(self.root), "-B", str(self.root / "build"), f"-DAGX_MOTOR_SDK_FETCH_PROTOCOL={fetch}"],
            env=environment, capture_output=True, text=True, timeout=30,
        )

    def test_fetch_failure_is_fatal_and_does_not_stage_old_library(self):
        result = self.configure("ON")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Protocol fetch failed (7)", result.stderr)
        self.assertFalse(self.staged.parent.exists())
        self.assertEqual(self.library.read_bytes(), b"unverified old library")

    def test_fetch_off_stages_local_library(self):
        result = self.configure("OFF")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.staged.read_bytes(), b"unverified old library")

    def test_explicit_skip_stages_local_library(self):
        result = self.configure("ON", skip=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.staged.read_bytes(), b"unverified old library")

    def test_exported_config_resolves_yaml_dependency_before_targets(self):
        (self.root / "agx_motor_sdkConfig.cmake").write_text(
            (self.cmake_dir / "agx_motor_sdkConfig.cmake.in").read_text()
        )
        (self.root / "agx_motor_sdkTargets.cmake").write_text(
            'if(NOT TARGET yaml-cpp)\nmessage(FATAL_ERROR "yaml-cpp dependency not resolved")\nendif()\n'
        )
        (self.root / "CMakeLists.txt").write_text(
            'cmake_minimum_required(VERSION 3.14)\nproject(config_probe LANGUAGES CXX)\n'
            'include("${CMAKE_CURRENT_SOURCE_DIR}/agx_motor_sdkConfig.cmake")\n'
        )
        result = self.configure("OFF")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()