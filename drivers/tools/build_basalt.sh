#!/bin/zsh
# SPDX-License-Identifier: MPL-2.0
#
# Build Basalt (Monado's fork) as the VIT plugin that gives a wired Windows
# Mixed Reality headset 6DoF head tracking, and print where libbasalt.dylib
# ended up. See docs/platforms/wmr.md, "6DoF head tracking".
#
#   drivers/tools/build_basalt.sh [SOURCE_DIR] [BUILD_DIR]
#
# Defaults: SOURCE_DIR=$HOME/Library/Caches/OXRSys/basalt (a shallow clone is
# made when missing), BUILD_DIR=$SOURCE_DIR/build. Needs Homebrew's
# eigen, tbb, fmt, opencv (the same OpenCV the driver library is built
# against), cmake and ninja. Only the shared library is built: no Pangolin,
# no ROS, no tests.
#
# Install the result next to oxrsys-headset-helper, or as
# ~/Library/Application Support/OXRSys/libbasalt.dylib; the helper looks in
# both places (or pass --vit-library PATH).

set -eu

BASALT_REPO="https://gitlab.freedesktop.org/mateosss/basalt.git"
BASALT_COMMIT="df6e970c8da7636eb401a09e3317fbeaaf829b9a"
# Submodules the shared library needs, pinned to what the commit above records.
SUBMODULES=(thirdparty/basalt-headers thirdparty/opengv thirdparty/magic_enum thirdparty/CLI11)
HEADER_SUBMODULES=(thirdparty/Sophus thirdparty/cereal thirdparty/eigen)

SRC="${1:-$HOME/Library/Caches/OXRSys/basalt}"
BUILD="${2:-$SRC/build}"

for tool in git cmake ninja; do
    if ! command -v "$tool" >/dev/null; then
        echo "error: $tool not found (brew install $tool)" >&2
        exit 1
    fi
done
for pkg in eigen tbb fmt opencv; do
    if [ ! -d "/opt/homebrew/opt/$pkg" ]; then
        echo "error: Homebrew package $pkg missing (brew install $pkg)" >&2
        exit 1
    fi
done

if [ ! -d "$SRC/.git" ]; then
    echo "==> Cloning Basalt into $SRC"
    mkdir -p "$(dirname "$SRC")"
    git clone --no-checkout "$BASALT_REPO" "$SRC"
fi
cd "$SRC"
git fetch --depth 1 origin "$BASALT_COMMIT"
git checkout -q "$BASALT_COMMIT"
echo "==> Fetching submodules"
git submodule update --init --depth 1 "${SUBMODULES[@]}"
(cd thirdparty/basalt-headers && git submodule update --init --depth 1 "${HEADER_SUBMODULES[@]}")

echo "==> Configuring in $BUILD"
cmake -S "$SRC" -B "$BUILD" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DBASALT_BUILD_SHARED_LIBRARY_ONLY=ON \
    -DBASALT_BUILD_VISUALIZATION=OFF \
    -DBASALT_ENABLE_ROSBAG2=OFF \
    -DBASALT_BUILD_TESTS=OFF \
    -DCMAKE_PREFIX_PATH="/opt/homebrew/opt/opencv;/opt/homebrew/opt/tbb;/opt/homebrew/opt/fmt"
echo "==> Building (this takes a few minutes)"
cmake --build "$BUILD" --target basalt

LIB="$BUILD/libbasalt.dylib"
if [ ! -e "$LIB" ]; then
    echo "error: $LIB was not produced" >&2
    exit 1
fi
echo
echo "Built $(readlink -f "$LIB")"
echo "Exports: $(nm -gU "$(readlink -f "$LIB")" | grep -c ' _vit_') vit_* functions"
echo
echo "Install it with, for example:"
echo "  cp \"$(readlink -f "$LIB")\" \"\$HOME/Library/Application Support/OXRSys/libbasalt.dylib\""
