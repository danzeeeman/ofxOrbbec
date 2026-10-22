# ofxOrbbec

An openFrameworks addon for Orbbec Femto Mega depth cameras, on [OrbbecSDK_v2](https://github.com/orbbec/OrbbecSDK_v2). It provides:

- **Colour and depth** from one camera, captured on a thread of its own, with the depth registered to the colour picture (on the camera when it can, in software otherwise).
- **Cameras on USB or on the network** (Ethernet / PoE). `ofxOrbbecFinder` finds them for you.
- **The accelerometer**, as an "up" vector in the colour camera's frame: a camera's pitch and roll without any calibration.
- **Several cameras in hardware sync** over the trigger cable (`ofxOrbbecManager`).
- **Recording and playback**: a `.femto` file holds colour, depth and the accelerometer frame by frame, and plays back through the same calls as a live camera.

It is used by [FemtoMegaSkeletonTracking](https://github.com/danzeeeman/FemtoMegaSkeletonTracking) (femtoSkeletonOsc).

## Install

Clone it into `<openFrameworks>/addons/ofxOrbbec`. The Orbbec SDK is not in this repository (`libs/OrbbecSDK/` is git-ignored). Put it there yourself.

### macOS

```sh
scripts/install_sdk.sh            # fetches OrbbecSDK_v2 v2.10.6 into libs/OrbbecSDK
scripts/install_sdk.sh --force    # replaces an SDK already there
```

FemtoMegaSkeletonTracking's `get_addons.sh` runs it for you. To do it by hand:

1. Download the macOS release of OrbbecSDK_v2 v2.10.6 from [its releases](https://github.com/orbbec/OrbbecSDK_v2/releases).
2. Lay it out like this:

   ```
   libs/OrbbecSDK/
       include/libobsensor/...
       lib/macos/libOrbbecSDK*.dylib
       extensions/            (from the release's lib/extensions)
       OrbbecSDKConfig.xml
       LICENSE.txt
   ```

`extensions/` stays outside `lib/` on purpose. openFrameworks links every dylib under `libs/*/lib/macos`, but these are plugins the SDK loads itself at runtime.

After the build, `addon_config.mk` copies the dylibs and `extensions/` into `bin/<app>.app/Contents/MacOS/`, next to the executable. It uses `rsync --ignore-existing`, so an app that copied (and signed) them itself keeps its copies.

Cameras on the network need two more things from the app:

- **The local network permission.** Put `NSLocalNetworkUsageDescription` in the app's `openFrameworks-Info.plist`; without it macOS quietly blocks discovery and connections.
- **A signature for the whole bundle.** A `make` build only signs the executable, so an app opened from Finder is not given the local network. Add this to the app's `config.make`:

  ```make
  PROJECT_AFTER += codesign --force --deep --sign - --entitlements of.entitlements bin/$(BIN_NAME).app ;
  ```

### Windows

`addon_config.mk` expects an extracted OrbbecSDK_v2 release at `C:/src/OrbbecSDK` (`include/`, `lib/OrbbecSDK.lib`). Copy `bin/OrbbecSDK.dll` and its `bin/extensions/` folder next to the built `.exe`. Change the paths in `addon_config.mk` if your SDK is somewhere else.

### Without the SDK

When `OFX_ORBBEC_HAVE_SDK` is not defined, the addon still compiles, against stubs: nothing opens and the finder finds nothing. The macOS section of `addon_config.mk` defines it.

## Use

### One camera

```cpp
// ofApp.h
#include "ofxOrbbecDevice.h"
ofxOrbbecDevice cam;
ofTexture colorTex;

// setup()
ofxOrbbecDevice::Options opt;
opt.alignDepthToColor = true;          // depth the same size as colour: colour pixel (u,v) indexes depth
opt.netAddress = "192.168.1.10";       // a camera on the network; leave empty for USB
cam.open("", ofxOrbbecDevice::SyncMode::Standalone, opt);
// on USB, open by serial number instead: cam.open("CL2H84100LE");

// update()
cam.update();
const ofxOrbbecFrame& f = cam.getFrame();
if (f.isNew) {
    colorTex.loadData(f.color);        // RGB
    uint16_t mm = f.depth[f.depth.getPixelIndex(320, 240)];   // depth in millimetres, 0 = none
}
glm::vec3 up;
if (cam.getUp(up)) { /* up in the colour camera's frame: x right, y down, z forward */ }
```

- **`Options`.**
  - Zero width, height and fps (the default) picks the fastest mode the camera offers: the highest frame rate, then the smallest image at that rate. Non-zero values narrow the choice first.
  - On the network the SDK only uses port 8090. Once a camera opens by address, `getSerial()` returns its serial number.
- **Intrinsics.** `getColorIntrinsics()` and `getDepthIntrinsics()` give the camera intrinsics: focal length, centre and distortion. With `alignDepthToColor` the two are the same.
- **Threads.** `update()` never waits. It swaps in the newest frame, if there is one. Call it from the main thread.

### Finding cameras

```cpp
#include "ofxOrbbecFinder.h"
ofxOrbbecFinder finder;

finder.start(3.f);                     // look every 3 s, on a thread of its own
for (auto& c : finder.getFound()) {    // safe from any thread
    // c.serial, c.name, c.connection ("USB", "Ethernet", ...), c.ip (empty on USB)
}
```

Network discovery only finds cameras on the computer's own subnet. Each Femto needs an address of its own; the factory address is `192.168.1.10`, and Orbbec Viewer can change it.

### Several cameras in sync

```cpp
#include "ofxOrbbecManager.h"
ofxOrbbecManager rig;

rig.openAll();                         // every connected camera; the first is sync primary, the rest secondary
// update()
rig.update();
for (size_t i = 0; i < rig.deviceCount(); i++) { auto& f = rig.device(i).getFrame(); }
```

This needs the cameras joined by the sync cable. Without one, open each camera on its own with `SyncMode::Standalone`.

### Recording and playback

```cpp
#include "ofxOrbbecRecording.h"
ofxOrbbecRecorder rec;
ofxOrbbecPlayer player;

// record
rec.start(ofToDataPath("take.femto", true), cam.getSerial(), cam.getColorIntrinsics());
glm::vec3 up; bool hasUp = cam.getUp(up);
if (cam.getFrame().isNew) rec.add(cam.getFrame().color, cam.getFrame().depth, hasUp, up);
rec.stop();

// play back: the same calls as a live camera
player.open(ofToDataPath("take.femto", true));
player.loop = true; player.speed = 1.f;
player.update();
const ofxOrbbecFrame& f = player.getFrame();
```

- **Recording.** `add()` only queues a copy of the frame. The compressing and writing happen on a thread of their own, so the camera is never held up. If the writer falls behind, new frames are dropped and counted (`getFramesDropped()`).
- **The file.** Colour is stored as JPEG (`jpegQuality`, 92 by default) and depth is zlib-compressed without loss. The format is described at the top of `ofxOrbbecRecording.h`.
- **Playback.** Frames come at the times they were recorded (times `speed`). `getPosition()`, `getLength()` and `isFinished()` tell you where it is.

## Files

| file | |
|---|---|
| `ofxOrbbecDevice` | one camera: open, threaded capture, depth registered to colour, accelerometer |
| `ofxOrbbecFinder` | finds cameras on USB and on the network, on its own thread |
| `ofxOrbbecManager` | several cameras with trigger-cable sync |
| `ofxOrbbecRecording` | `ofxOrbbecRecorder` and `ofxOrbbecPlayer`, the `.femto` format |
| `ofxOrbbecTypes.h` | `ofxOrbbecFrame` (depth in mm, RGB colour, timestamp) and `ofxOrbbecIntrinsics` |

## License

The Orbbec SDK has its own license (`libs/OrbbecSDK/LICENSE.txt`, from its release).
