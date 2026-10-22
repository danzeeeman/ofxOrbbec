#!/usr/bin/env bash
# Fetches the OrbbecSDK_v2 release that ofxOrbbec links and lays it out as addon_config.mk expects:
#
#     libs/OrbbecSDK/{include, lib/macos/*.dylib, extensions/, OrbbecSDKConfig.xml, LICENSE.txt}
#
# macOS only for now (Windows: see the README). An SDK already there is left alone; pass --force to replace it.
# OFX_ORBBEC_SDK_DIR puts it somewhere else (for testing).
set -euo pipefail

version="2.10.6"
url="https://github.com/orbbec/OrbbecSDK_v2/releases/download/v${version}/OrbbecSDK_v2.10.6_202609291804_575347c_macOS.tar.gz"

here="$(cd "$(dirname "$0")" && pwd)"
sdk="${OFX_ORBBEC_SDK_DIR:-$here/../libs/OrbbecSDK}"

if [ "$(uname)" != "Darwin" ]; then
    echo "install_sdk.sh fetches the macOS SDK only. On Windows, extract an OrbbecSDK_v2 release to C:/src/OrbbecSDK (see README.md)." >&2
    exit 1
fi

if [ -e "$sdk/lib/macos/libOrbbecSDK.dylib" ] && [ "${1:-}" != "--force" ]; then
    echo "have    OrbbecSDK at $sdk (pass --force to replace it)"
    exit 0
fi

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

echo "fetch   OrbbecSDK v${version}"
curl -fL --progress-bar -o "$tmp/sdk.tar.gz" "$url"
tar -xzf "$tmp/sdk.tar.gz" -C "$tmp"

lib="$(find "$tmp" -name libOrbbecSDK.dylib -path '*/lib/*' | head -1)"
if [ -z "$lib" ]; then
    echo "no libOrbbecSDK.dylib in the download: has the release's layout changed?" >&2
    exit 1
fi
src="$(cd "$(dirname "$lib")/.." && pwd)"

rm -rf "$sdk"
mkdir -p "$sdk/lib/macos"
cp -R "$src/include" "$sdk/"
cp -a "$src/lib/"*.dylib "$sdk/lib/macos/"
# kept out of lib/: openFrameworks links every dylib under lib/macos, and these are plugins the SDK loads itself
cp -R "$src/lib/extensions" "$sdk/"
cp "$src/lib/OrbbecSDKConfig.xml" "$src/LICENSE.txt" "$sdk/"

echo "done    OrbbecSDK v${version} in $(cd "$sdk" && pwd)"
