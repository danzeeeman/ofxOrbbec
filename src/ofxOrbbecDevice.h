// ofxOrbbecDevice.h — single Orbbec Femto Mega, threaded capture
#pragma once
#include "ofMain.h"
#include "ofxOrbbecTypes.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <string>

// The real OrbbecSDK_v2 headers are not present in this environment. Once
// the vendor SDK is placed at addons/ofxOrbbec/libs/OrbbecSDK/ (see
// addon_config.mk) and OFX_ORBBEC_HAVE_SDK is defined for the build, this
// switches to the real ob:: types from <libobsensor/ObSensor.hpp>.
#ifdef OFX_ORBBEC_HAVE_SDK
#include <libobsensor/ObSensor.hpp>
#else
namespace ob {
    class Device;
    class Pipeline;
}
#endif

// Wraps a single Orbbec Femto Mega. API shape mirrors ofxKinect/
// ofxBase3DVideo (depth pixels, distance, textures) but generalized for
// multi-device hardware sync. Capture runs on a background thread; update()
// swaps in the latest frame under a mutex-guarded double buffer, the same
// threading shape ofxOvrtx uses for its render thread.
class ofxOrbbecDevice {
public:
    ~ofxOrbbecDevice() { close(); }

    enum class SyncMode { Standalone, Master, Subordinate };

    // serial identifies the physical device (Orbbec device serial number).
    // syncMode selects the hardware trigger-cable role for multi-device
    // capture; ofxOrbbecManager::openAll() sets this consistently across
    // its owned devices.
    bool open(const std::string& serial, SyncMode syncMode = SyncMode::Standalone);
    void close();
    bool isOpen() const { return isOpen_; }

    // Non-blocking; call every frame from the main thread. Swaps in the
    // latest captured frame if one is ready.
    void update();

    const ofxOrbbecFrame& getFrame() const { return front_; }
    const ofxOrbbecIntrinsics& getDepthIntrinsics() const { return depthIntrinsics_; }
    const ofxOrbbecIntrinsics& getColorIntrinsics() const { return colorIntrinsics_; }

    const std::string& getSerial() const { return serial_; }
    SyncMode getSyncMode() const { return syncMode_; }

private:
    void captureLoop();

    std::string serial_;
    SyncMode syncMode_ = SyncMode::Standalone;
    std::atomic<bool> isOpen_{false};

    std::thread captureThread_;
    std::atomic<bool> running_{false};

    std::mutex frameMutex_;
    ofxOrbbecFrame back_;
    ofxOrbbecFrame front_;
    std::atomic<bool> frameReady_{false};

    ofxOrbbecIntrinsics depthIntrinsics_;
    ofxOrbbecIntrinsics colorIntrinsics_;

#ifdef OFX_ORBBEC_HAVE_SDK
    std::shared_ptr<ob::Pipeline> pipeline_;
    std::shared_ptr<ob::Device>   device_;
#endif
};
