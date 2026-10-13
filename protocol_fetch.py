"""Download prebuilt arm protocol library before pip install."""
import hashlib
import json
import os
import platform
import re
import sys
import tempfile
from pathlib import Path
from typing import Any, Dict, Optional, Tuple
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlsplit
from urllib.request import HTTPRedirectHandler, Request, build_opener, urlopen

_ROOT = Path(__file__).resolve().parent
_SHA256_MARKER = "AGX_MOTOR_SDK_PROTOCOL_SHA256"
_VARIANTS = ("arm",)


def _parse_version_py() -> Tuple[str, str]:
    text = (_ROOT / "agx_motor_sdk" / "version.py").read_text(encoding="utf-8")
    sdk_version: Optional[str] = None
    protocol_version: Optional[str] = None
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("__protocol_version__"):
            protocol_version = line.split("=", 1)[1].strip().strip('"').strip("'")
        elif line.startswith("__version__"):
            sdk_version = line.split("=", 1)[1].strip().strip('"').strip("'")
    if not sdk_version:
        raise RuntimeError("cannot read __version__ from agx_motor_sdk/version.py")
    if not protocol_version:
        protocol_version = sdk_version
    return sdk_version, protocol_version


def _read_protocol_version() -> str:
    env = os.environ.get("AGX_MOTOR_SDK_PROTOCOL_VERSION", "").strip()
    if env:
        return env
    _, protocol_version = _parse_version_py()
    return protocol_version


def _host_platform_arch() -> Tuple[str, str]:
    pl = sys.platform
    if pl == "linux":
        p = "linux"
    elif pl == "darwin":
        p = "darwin"
    elif pl == "win32":
        p = "windows"
    else:
        p = pl.replace(" ", "_")
    m = platform.machine().lower()
    if m in ("aarch64", "arm64"):
        a = "aarch64"
    elif m in ("x86_64", "amd64"):
        a = "x86_64"
    elif m in ("i386", "i686"):
        a = "x86"
    else:
        a = m.replace(" ", "_")
    return p, a


def _dest_filename(variant: str) -> str:
    if sys.platform == "win32":
        return f"libagx_motor_protocol_{variant}.dll"
    if sys.platform == "darwin":
        return f"libagx_motor_protocol_{variant}.dylib"
    return f"libagx_motor_protocol_{variant}.so"


def _release_asset_name(variant: str, plat: str, arch: str, version: str) -> str:
    ext = {"windows": "dll", "darwin": "dylib"}.get(plat, "so")
    return f"agx-{variant}-can-v{version}-{plat}-{arch}.{ext}"


def _download_url(repo: str, tag: str, asset_name: str) -> str:
    return f"https://github.com/{repo}/releases/download/{quote(tag, safe='')}/{quote(asset_name, safe='')}"


def _sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


class _NoAuthRedirectHandler(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        redirected = super().redirect_request(req, fp, code, msg, headers, newurl)
        if redirected is not None:
            redirected.remove_header("Authorization")
        return redirected


def _github_token() -> Optional[str]:
    for name in ("AGX_MOTOR_SDK_GITHUB_TOKEN", "GH_TOKEN", "GITHUB_TOKEN"):
        token = os.environ.get(name, "").strip()
        if token:
            return token
    return None


def _api_bytes(url: str, token: Optional[str], accept: str, timeout: int) -> bytes:
    parsed = urlsplit(url)
    if parsed.scheme != "https" or parsed.netloc != "api.github.com":
        raise RuntimeError("Refusing protocol API request outside api.github.com.")
    req = Request(url, headers={"Accept": accept, "User-Agent": "agx_motor_sdk-setup"})
    if token:
        req.add_unredirected_header("Authorization", f"Bearer {token}")
        opener = build_opener(_NoAuthRedirectHandler())
        with opener.open(req, timeout=timeout) as resp:
            return resp.read()
    with urlopen(req, timeout=timeout) as resp:
        return resp.read()


def _fetch_release_metadata(repo: str, tag: str, token: Optional[str]) -> Dict[str, Any]:
    api = f"https://api.github.com/repos/{repo}/releases/tags/{quote(tag, safe='')}"
    try:
        data = json.loads(_api_bytes(api, token, "application/vnd.github+json", 30).decode("utf-8"))
        if not isinstance(data, dict):
            raise ValueError("release metadata is not an object")
        return data
    except Exception:
        if token:
            raise RuntimeError(
                f"Cannot install agx_motor_sdk: authenticated GitHub release metadata request failed for {repo} tag {tag}. "
                "Check token access and the release tag."
            ) from None
        raise RuntimeError(
            f"Cannot install agx_motor_sdk: GitHub release metadata request failed for {repo} tag {tag}. "
            "Verified release metadata is required; use AGX_MOTOR_SDK_SKIP_DOWNLOAD=1 for offline installation."
        ) from None


def _parse_sha256_map(body: str) -> Dict[str, Any]:
    if not body:
        return {}
    m = re.search(
        rf"<!--\s*{_SHA256_MARKER}\s*-->\s*```(?:json)?\s*([\s\S]*?)\s*```",
        body,
    )
    if not m:
        m = re.search(
            r"<!--\s*AGXMOTOR_CTRL_LIBMOTOR_SHA256\s*-->\s*```(?:json)?\s*([\s\S]*?)\s*```",
            body,
        )
    if not m:
        return {}
    try:
        out = json.loads(m.group(1).strip())
        return out if isinstance(out, dict) else {}
    except ValueError:
        return {}


def _http_get_bytes(url: str) -> bytes:
    with urlopen(url, timeout=120) as resp:
        return resp.read()


def _raise_network_error(cause: Optional[BaseException] = None) -> None:
    raise RuntimeError(
        "Cannot install agx_motor_sdk: failed to download protocol libraries. "
        "Set AGX_MOTOR_SDK_SKIP_DOWNLOAD=1 and place libraries under "
        "agx_motor_sdk/protocol/, or set AGX_MOTOR_SDK_ARM_LIB at runtime."
    ) from cause


def _raise_404(tag: str, asset: str, plat: str, arch: str, cause: Optional[BaseException] = None) -> None:
    raise RuntimeError(
        f"Cannot install agx_motor_sdk: asset '{asset}' not found in protocol "
        f"release {tag} ({plat}/{arch})."
    ) from cause


def _download_bytes(url: str, protocol_version: str, plat: str, arch: str, asset: str) -> bytes:
    try:
        data = _http_get_bytes(url)
    except HTTPError as e:
        if getattr(e, "code", None) == 404:
            _raise_404(protocol_version, asset, plat, arch, e)
        _raise_network_error(e)
    except (URLError, OSError) as e:
        _raise_network_error(e)
    if not data:
        raise RuntimeError(f"Downloaded {asset} is empty.")
    return data


def _ensure_variant(
    variant: str,
    plat: str,
    arch: str,
    protocol_version: str,
    repo: str,
    sha_map: Dict[str, Any],
    dest_dir: Path,
    tag: str,
    metadata: Dict[str, Any],
    token: Optional[str],
) -> None:
    dest_file = dest_dir / _dest_filename(variant)
    asset = _release_asset_name(variant, plat, arch, protocol_version)
    url = _download_url(repo, tag, asset)
    has_local = dest_file.is_file() and dest_file.stat().st_size > 0

    expected = sha_map.get(asset)
    if not isinstance(expected, str) or re.fullmatch(r"[0-9a-fA-F]{64}", expected) is None:
        raise RuntimeError(f"Missing or invalid SHA256 in release metadata for {asset}; a 64-hex digest is required.")
    expected = expected.lower()

    if has_local and _sha256_file(dest_file).lower() == expected:
        print(f"agx_motor_sdk: local {dest_file.name} matches release metadata, skip download.")
        return

    if not has_local or _sha256_file(dest_file).lower() != expected:
        if token:
            assets = metadata.get("assets", [])
            if not isinstance(assets, list):
                raise RuntimeError("Invalid authenticated GitHub release asset metadata.")
            matched = next((item for item in assets if isinstance(item, dict) and item.get("name") == asset), None)
            if matched is None:
                _raise_404(tag, asset, plat, arch)
            url = matched.get("url")
            if not isinstance(url, str) or not re.fullmatch(
                rf"https://api\.github\.com/repos/{re.escape(repo)}/releases/assets/[0-9]+", url
            ):
                raise RuntimeError("Refusing untrusted GitHub release asset API URL.")
        print(f"agx_motor_sdk: downloading {asset}: {url}")
        if token:
            try:
                data = _api_bytes(url, token, "application/octet-stream", 120)
            except Exception:
                raise RuntimeError(f"Authenticated GitHub asset download failed for {asset}; check token access.") from None
            if not data:
                raise RuntimeError(f"Downloaded {asset} is empty.")
        else:
            data = _download_bytes(url, tag, plat, arch, asset)
        if hashlib.sha256(data).hexdigest() != expected:
            raise RuntimeError(f"SHA256 mismatch for {asset}; existing library was not replaced.")
        dest_dir.mkdir(parents=True, exist_ok=True)
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(dir=dest_dir, prefix=f".{dest_file.name}.", delete=False) as temporary:
                temporary_path = Path(temporary.name)
                temporary.write(data)
                temporary.flush()
                os.fsync(temporary.fileno())
            os.replace(temporary_path, dest_file)
        finally:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)
        print(f"agx_motor_sdk: wrote {dest_file} ({len(data)} bytes).")
        return

    print(f"agx_motor_sdk: using local {dest_file.name}")


def download_protocol_libs_if_needed() -> None:
    if os.environ.get("AGX_MOTOR_SDK_SKIP_DOWNLOAD", "").strip():
        print("agx_motor_sdk: AGX_MOTOR_SDK_SKIP_DOWNLOAD set, skip protocol download.")
        return

    plat, arch = _host_platform_arch()
    dest_dir = _ROOT / "agx_motor_sdk" / "protocol"
    protocol_version = _read_protocol_version()
    repo = os.environ.get("AGX_MOTOR_SDK_GITHUB_REPO", "kehuanjack/agx_motor_protocol_releases").strip()
    if not re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", repo):
        raise RuntimeError("AGX_MOTOR_SDK_GITHUB_REPO must be a GitHub owner/repo.")
    tag = os.environ.get("AGX_MOTOR_SDK_PROTOCOL_TAG", "").strip() or f"arm-can-v{protocol_version}"
    token = _github_token()
    metadata = _fetch_release_metadata(repo, tag, token)
    body = metadata.get("body")
    sha_map = _parse_sha256_map(body if isinstance(body, str) else "")

    for variant in _VARIANTS:
        _ensure_variant(variant, plat, arch, protocol_version, repo, sha_map, dest_dir, tag, metadata, token)


def get_cmdclass() -> Dict[str, Any]:
    from setuptools.command.build_py import build_py as _build_py
    from setuptools.command.develop import develop as _develop

    class _ProtocolBuildPy(_build_py):
        def run(self):
            download_protocol_libs_if_needed()
            super().run()

    class _ProtocolDevelop(_develop):
        def run(self):
            download_protocol_libs_if_needed()
            super().run()

    return {"build_py": _ProtocolBuildPy, "develop": _ProtocolDevelop}


if __name__ == "__main__":
    download_protocol_libs_if_needed()
