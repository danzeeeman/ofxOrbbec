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
    // Stream options. Zero width/height/fps = the fastest mode the device offers: the highest frame
    // rate, then the smallest image at that rate (any non-zero values narrow the choice first).
    // alignDepthToColor registers depth into the color camera (SDK Align filter), so
    // depth and color are the same size and a color pixel (u,v) indexes depth directly;
    // depth intrinsics then equal the color intrinsics.
    struct Options {
        int  colorWidth = 0;
        int  colorHeight = 0;
        int  colorFps = 0;
        int  depthWidth = 0;
        int  depthHeight = 0;
        int  depthFps = 0;
        bool alignDepthToColor = false;
        // A Femto Mega on the network (Ethernet): its IPv4 address, e.g. "192.168.1.10"; the SDK only
        // supports port 8090. When set, open() connects to it directly and `serial` is not used to find it
        // (it is filled in from the device).
        std::string netAddress;
        int netPort = 8090;
    };

    // No `= Options()` default argument: clang rejects it while Options' member initializers are still pending.
    bool open(const std::string& serial, SyncMode syncMode = SyncMode::Standalone) { return open(serial, syncMode, Options()); }
    bool open(const std::string& serial, SyncMode syncMode, const Options& options);
    void close();
    bool isOpen() const { return isOpen_; }

    // Non-blocking; call every frame from the main thread. Swaps in the
    // latest captured frame if one is ready.
    void update();

    const ofxOrbbecFrame& getFrame() const { return front_; }
    const ofxOrbbecIntrinsics& getDepthIntrinsics() const { return depthIntrinsics_; }
    const ofxOrbbecIntrinsics& getColorIntrinsics() const { return colorIntrinsics_; }

    // The accelerometer: which way is up, as a unit vector in the colour camera's frame (x right, y down, z
    // forward), low-passed. It is the base pose of the camera (its pitch and roll) without any calibration.
    // False when the device has no accelerometer, or it has not reported yet.
    bool getUp(glm::vec3& up) const {
        if (!imuOk_) return false;
        up = glm::vec3(upX_.load(), upY_.load(), upZ_.load());
        return true;
    }
    bool hasImu() const { return imuOk_; }

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

    // accelerometer
    std::atomic<bool> imuOk_{false};
    std::atomic<float> upX_{0.f}, upY_{-1.f}, upZ_{0.f};
    float imuRot_[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};   // accelerometer axes -> colour camera axes
    glm::vec3 imuUp_{0.f, 1.f, 0.f};                    // low-pass state, accelerometer frame
    bool imuPrimed_ = false;

    ofxOrbbecIntrinsics depthIntrinsics_;
    ofxOrbbecIntrinsics colorIntrinsics_;

#ifdef OFX_ORBBEC_HAVE_SDK
    std::shared_ptr<ob::Pipeline> pipeline_;
    std::shared_ptr<ob::Device>   device_;
    std::shared_ptr<ob::Align>    align_;
    std::shared_ptr<ob::Sensor>   accelSensor_;
#endif
    Options options_;
};
