#!/usr/bin/env bash
# Bootstrap vcpkg into vendor/vcpkg (run once before first cmake configure)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
VCPKG_PATH="${1:-$ROOT_DIR/vendor/vcpkg}"

if [ -f "$VCPKG_PATH/vcpkg" ]; then
    echo "vcpkg already bootstrapped at $VCPKG_PATH"
elif [ -d "$VCPKG_PATH" ]; then
    echo "Bootstrapping existing clone at $VCPKG_PATH ..."
    "$VCPKG_PATH/bootstrap-vcpkg.sh" -disableMetrics
else
    echo "Cloning vcpkg into $VCPKG_PATH ..."
    git clone https://github.com/microsoft/vcpkg.git "$VCPKG_PATH"
    echo "Bootstrapping..."
    "$VCPKG_PATH/bootstrap-vcpkg.sh" -disableMetrics
fi

echo ""
echo "Done. To configure and build:"
echo "  export VCPKG_ROOT='$VCPKG_PATH'"
echo "  cmake --preset macos-clang-release"
echo "  cmake --build build/macos-clang-release"
