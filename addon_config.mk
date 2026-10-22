meta:
	ADDON_NAME = ofxOrbbec
	ADDON_DESCRIPTION = Addon for interfacing with Orbbec Femto Mega depth cameras (OrbbecSDK_v2), N-device hardware-synced capture
	ADDON_AUTHOR = danm
	ADDON_TAGS = "computer vision" "3D sensing" "orbbec" "femto mega" "depth camera"
	ADDON_URL = https://github.com/orbbec/OrbbecSDK_v2

common:
	# OrbbecSDK_v2 is not vendored in this repo. On this machine it's already
	# extracted (prebuilt release, headers verified line-by-line against this
	# exact local checkout) at C:/src/OrbbecSDK - absolute paths below, same
	# convention as ofxOvrtx's addon_config.mk pointing at C:/src/ovrtx and
	# ofxNvblox's pointing at C:/src/nvblox. There is no fallback discovery.
	# On another machine without that checkout, extract an OrbbecSDK_v2
	# release and update these paths (or fall back to a libs/OrbbecSDK/
	# in-addon layout) - until real paths resolve, ofxOrbbecDevice/
	# ofxOrbbecManager compile against their stub (non-SDK) code path (see
	# OFX_ORBBEC_HAVE_SDK in ofxOrbbecDevice.h). Runtime needs
	# C:/src/OrbbecSDK/bin/OrbbecSDK.dll + the bin/extensions/ subfolder
	# (depthengine/filters/firmwareupdater/frameprocessor) copied alongside
	# the built .exe.

vs:
	ADDON_INCLUDES = C:/src/OrbbecSDK/include
	ADDON_CFLAGS = /std:c++17
	# Single-config .lib (no separate Debug build shipped in this release) -
	# safe to link from both Debug and Release configs, unlike cuMotion/
	# nvblox's Release-only /MD constraint. This addon_config is
	# documentation for non-VS platforms / projectGenerator use; the actual
	# Windows build (apps/calibratedDepth/{reconstructor,calibrationTool}.vcxproj)
	# hand-wires these same paths directly, matching the ofxOvrtx/
	# robotController.vcxproj precedent (Windows doesn't consume
	# addon_config.mk).
	ADDON_LIBS = C:/src/OrbbecSDK/lib/OrbbecSDK.lib
