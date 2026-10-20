#!/usr/bin/env bash
# Remove build artifacts and caches (matches .gitignore; keeps venv / .venv)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

_prune() {
  find "$ROOT" \
    '(' -path '*/.git' -o -path '*/.git/*' \
       -o -path '*/.venv' -o -path '*/.venv/*' \
       -o -path '*/venv' -o -path '*/venv/*' ')' -prune -o \
    "$@" -print0 2>/dev/null || true
}

remove_paths() {
  while IFS= read -r -d '' path; do
    echo "rm -rf -- $path"
    rm -rf -- "$path"
  done
}

remove_files() {
  while IFS= read -r -d '' path; do
    echo "rm -f -- $path"
    rm -f -- "$path"
  done
}

echo "==> Cleaning build artifacts: $ROOT"

# CMake / setuptools output
remove_paths < <(_prune -type d '(' \
  -name build -o -name dist -o -name .agx_motor_sdk_build \
  -o -name install -o -name log -o -name .cache ')')

# Prebuilt protocol libraries (download → agx_motor_sdk/protocol/;
# cmake stage copy → build/protocol/)
remove_files < <(_prune -type f '(' \
  -path '*/agx_motor_sdk/protocol/*.so' \
  -o -path '*/agx_motor_sdk/protocol/*.dll' \
  -o -path '*/agx_motor_sdk/protocol/*.dylib' \
  -o -path '*/build/protocol/*.so' \
  -o -path '*/build/protocol/*.dll' \
  -o -path '*/build/protocol/*.dylib' ')')

# Legacy repo-root protocol/ (duplicate; should not exist)
if [[ -d "$ROOT/protocol" ]]; then
  echo "rm -rf -- $ROOT/protocol"
  rm -rf -- "$ROOT/protocol"
fi

# pybind / CMake generated files
remove_files < <(_prune -type f -name compile_commands.json)

# Python packaging
remove_files < <(_prune -type f '(' -name '*.whl' -o -name '*.egg' ')')
remove_paths < <(_prune -type d -name '*.egg-info')
remove_paths < <(_prune -type d -name .eggs)

# Python bytecode cache
remove_paths < <(_prune -type d -name __pycache__)
remove_files < <(_prune -type f '(' -name '*.pyc' -o -name '*.pyo' ')')

echo "==> Done"
