#include "ofxOrbbecDevice.h"

#ifdef OFX_ORBBEC_HAVE_SDK
namespace {
struct VideoMode {
    uint32_t width = 0, height = 0, fps = 0;
};

// The fastest of a sensor's modes that match the non-zero width/height/fps: highest fps, then fewest
// pixels. Formats are ignored here; the caller asks for the one it wants and the SDK converts.
VideoMode pickFastest(const std::shared_ptr<ob::StreamProfileList>& list, int width, int height, int fps) {
    VideoMode best;
    if (!list) return best;
    for (uint32_t i = 0; i < list->getCount(); ++i) {
        auto p = list->getProfile(i);
        if (!p || !p->is<ob::VideoStreamProfile>()) continue;
        auto v = p->as<ob::VideoStreamProfile>();
        const uint32_t w = v->getWidth(), h = v->getHeight(), f = v->getFps();
        if ((width > 0 && (int)w != width) || (height > 0 && (int)h != height) || (fps > 0 && (int)f != fps)) continue;
        if (f > best.fps || (f == best.fps && (uint64_t)w * h < (uint64_t)best.width * best.height)) best = {w, h, f};
    }
    return best;
}
}
#endif

bool ofxOrbbecDevice::open(const std::string& serial, SyncMode syncMode, const Options& options) {
    if (isOpen_) {
        ofLogWarning("ofxOrbbecDevice") << "already open (" << serial_ << ")";
        return false;
    }

    serial_ = serial;
    syncMode_ = syncMode;
    options_ = options;

#ifdef OFX_ORBBEC_HAVE_SDK
    try {
        ob::Context context;
        if (!options_.netAddress.empty()) {
            device_ = context.createNetDevice(options_.netAddress.c_str(), (uint16_t)options_.netPort);
            if (!device_) {
                ofLogWarning("ofxOrbbecDevice") << "no device at " << options_.netAddress << ":" << options_.netPort;
                return false;
            }
            serial_ = device_->getDeviceInfo()->getSerialNumber();
        } else {
            auto deviceList = context.queryDeviceList();
            device_ = deviceList->getDeviceBySN(serial.c_str());
            if (!device_) {
                ofLogWarning("ofxOrbbecDevice") << "device not found: " << serial;
                return false;
            }
        }

        // Hardware trigger-cable sync role. Verified enum:
        // OB_MULTI_DEVICE_SYNC_MODE_{PRIMARY,SECONDARY,SECONDARY_SYNCED,STANDALONE,
        // SOFTWARE_TRIGGERING,HARDWARE_TRIGGERING,FREE_RUN}. Subordinate maps to
        // SECONDARY_SYNCED (subordinate that free-runs until synced by the trigger
        // signal, matching a cabled multi-Femto-Mega rig) rather than plain
        // SECONDARY — confirm this is the desired mode for the real trigger-cable
        // topology once hardware is in hand.
        OBMultiDeviceSyncConfig syncConfig = device_->getMultiDeviceSyncConfig();
        switch (syncMode) {
            case SyncMode::Master:
                syncConfig.syncMode = OB_MULTI_DEVICE_SYNC_MODE_PRIMARY;
                break;
            case SyncMode::Subordinate:
                syncConfig.syncMode = OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED;
                break;
            case SyncMode::Standalone:
            default:
                syncConfig.syncMode = OB_MULTI_DEVICE_SYNC_MODE_STANDALONE;
                break;
        }
        // TODO(Milestone 2): tune depthDelayUs/colorDelayUs/trigger2ImageDelayUs/
        // triggerOutEnable/triggerOutDelayUs/framesPerTrigger for the actual rig's
        // cable topology — left at getMultiDeviceSyncConfig()'s device defaults here.
        device_->setMultiDeviceSyncConfig(syncConfig);

        pipeline_ = std::make_shared<ob::Pipeline>(device_);
        auto config = std::make_shared<ob::Config>();
        // Pick explicit modes rather than OB_*_ANY: left to itself the SDK can choose a slower one
        // (e.g. the 15 fps wide depth mode), which then holds back the whole synced frameset.
        const VideoMode depthMode = pickFastest(pipeline_->getStreamProfileList(OB_SENSOR_DEPTH),
                                                options_.depthWidth, options_.depthHeight, options_.depthFps);
        const VideoMode colorMode = pickFastest(pipeline_->getStreamProfileList(OB_SENSOR_COLOR),
                                                options_.colorWidth, options_.colorHeight, options_.colorFps);
        if (depthMode.fps == 0 || colorMode.fps == 0) {
            ofLogWarning("ofxOrbbecDevice") << serial_ << ": no " << (depthMode.fps == 0 ? "depth" : "colour")
                                          << " mode matches the requested width/height/fps";
            return false;
        }
        ofLogNotice("ofxOrbbecDevice") << serial_ << ": depth " << depthMode.width << "x" << depthMode.height << " @ " << depthMode.fps
                                       << " fps, colour " << colorMode.width << "x" << colorMode.height << " @ " << colorMode.fps << " fps";
        config->enableVideoStream(OB_STREAM_DEPTH, depthMode.width, depthMode.height, depthMode.fps, OB_FORMAT_Y16);
        config->enableVideoStream(OB_STREAM_COLOR, colorMode.width, colorMode.height, colorMode.fps, OB_FORMAT_RGB);
        align_.reset();
        if (options_.alignDepthToColor) {
            // Depth registered into the colour camera with the calibration the device carries. On the device
            // (hardware D2C) when it can do it for these two modes: free, and no frames lost to it. Otherwise the
            // SDK's Align filter, on the capture thread, with the same calibration read from the device.
            bool hw = false;
            try {
                auto cp = pipeline_->getStreamProfileList(OB_SENSOR_COLOR)->getVideoStreamProfile(colorMode.width, colorMode.height, OB_FORMAT_ANY, colorMode.fps);
                auto dl = cp ? pipeline_->getD2CDepthProfileList(cp, ALIGN_D2C_HW_MODE) : nullptr;
                for (uint32_t i = 0; dl && i < dl->getCount() && !hw; ++i) {
                    auto p = dl->getProfile(i);
                    if (!p || !p->is<ob::VideoStreamProfile>()) continue;
                    auto v = p->as<ob::VideoStreamProfile>();
                    hw = v->getWidth() == depthMode.width && v->getHeight() == depthMode.height && v->getFps() == depthMode.fps;
                }
            } catch (const ob::Error& e) {
                ofLogNotice("ofxOrbbecDevice") << serial_ << ": no hardware D2C (" << e.getMessage() << ")";
            }
            if (hw) {
                config->setAlignMode(ALIGN_D2C_HW_MODE);
            } else {
                align_ = std::make_shared<ob::Align>(OB_STREAM_COLOR);
            }
            ofLogNotice("ofxOrbbecDevice") << serial_ << ": depth registered to colour " << (hw ? "on the device (hardware D2C)" : "in software (SDK Align filter)");
        }
        // Depth and colour paired by time, and only framesets that have both: registering one moment's depth
        // into another moment's colour puts a moving person's depth beside them.
        config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
        try {
            pipeline_->enableFrameSync();
        } catch (const ob::Error& e) {
            ofLogWarning("ofxOrbbecDevice") << serial_ << ": no frame sync (" << e.getMessage() << "); depth and colour may be a frame apart";
        }
        pipeline_->start(config);

        // One synchronous frameset to read back the active stream profiles'
        // calibration (intrinsics/distortion aren't queryable before streaming
        // starts in the examples this was verified against).
        // The first framesets can arrive without a colour frame (a networked camera starts depth first), and
        // the intrinsics below come from it, so wait for one that has both.
        std::shared_ptr<ob::FrameSet> frameSet;
        const uint64_t waitUntil = ofGetElapsedTimeMillis() + 3000;
        while (ofGetElapsedTimeMillis() < waitUntil) {
            auto fs = pipeline_->waitForFrameset(500);
            if (fs && fs->getFrame(OB_FRAME_COLOR) && fs->getFrame(OB_FRAME_DEPTH)) {
                frameSet = fs;
                break;
            }
        }
        if (!frameSet) {
            ofLogWarning("ofxOrbbecDevice") << "timed out waiting for first frameset: " << serial;
            pipeline_->stop();
            return false;
        }
        // The accelerometer, if the device has one: a gravity direction is the camera's pitch and roll.
        // Optional: a failure here only means no IMU.
        try {
            auto accel = device_->getSensor(OB_SENSOR_ACCEL);
            auto colorRaw = frameSet->getFrame(OB_FRAME_COLOR);
            if (accel && colorRaw) {
                auto profile = accel->getStreamProfileList()->getProfile(OB_PROFILE_DEFAULT);
                OBExtrinsic ex = profile->getExtrinsicTo(colorRaw->getStreamProfile());
                for (int i = 0; i < 9; ++i) imuRot_[i] = ex.rot[i];
                imuPrimed_ = false;
                imuOk_ = false;
                accel->start(profile, [this](std::shared_ptr<ob::Frame> frame) {
                    auto af = frame ? frame->as<ob::AccelFrame>() : nullptr;
                    if (!af) return;
                    const OBAccelValue v = af->getValue();
                    glm::vec3 a(v.x, v.y, v.z);
                    const float len = glm::length(a);
                    if (len < 6.f || len > 14.f) return;   // being shaken, not just held: not a gravity reading
                    a /= len;
                    imuUp_ = imuPrimed_ ? glm::normalize(glm::mix(imuUp_, a, 0.02f)) : a;
                    imuPrimed_ = true;
                    // accelerometer axes -> colour camera axes (rotation, row-major)
                    const glm::vec3 c(imuRot_[0] * imuUp_.x + imuRot_[1] * imuUp_.y + imuRot_[2] * imuUp_.z,
                                      imuRot_[3] * imuUp_.x + imuRot_[4] * imuUp_.y + imuRot_[5] * imuUp_.z,
                                      imuRot_[6] * imuUp_.x + imuRot_[7] * imuUp_.y + imuRot_[8] * imuUp_.z);
                    upX_ = c.x; upY_ = c.y; upZ_ = c.z;
                    imuOk_ = true;
                });
                accelSensor_ = accel;
                ofLogNotice("ofxOrbbecDevice") << "accelerometer streaming";
            } else {
                ofLogNotice("ofxOrbbecDevice") << "no accelerometer on this device";
            }
        } catch (const ob::Error& e) {
            ofLogWarning("ofxOrbbecDevice") << "accelerometer not available: " << e.getMessage();
            accelSensor_.reset();
        }
        auto readIntrinsics = [](const std::shared_ptr<ob::Frame>& frame) -> ofxOrbbecIntrinsics {
            ofxOrbbecIntrinsics out;
            if (!frame) return out;
            auto profile = frame->getStreamProfile()->as<ob::VideoStreamProfile>();
            OBCameraIntrinsic intr = profile->getIntrinsic();
            OBCameraDistortion dist = profile->getDistortion();
            out.width = intr.width; out.height = intr.height;
            out.fx = intr.fx; out.fy = intr.fy; out.cx = intr.cx; out.cy = intr.cy;
            out.k1 = dist.k1; out.k2 = dist.k2; out.k3 = dist.k3;
            out.p1 = dist.p1; out.p2 = dist.p2;
            return out;
        };
        colorIntrinsics_ = readIntrinsics(frameSet->getFrame(OB_FRAME_COLOR));
        if (options_.alignDepthToColor) {
            // depth is registered into the color camera (on the device or by align_): same pixel grid, same intrinsics
            depthIntrinsics_ = colorIntrinsics_;
        } else {
            depthIntrinsics_ = readIntrinsics(frameSet->getFrame(OB_FRAME_DEPTH));
        }
    } catch (const ob::Error& e) {
        ofLogWarning("ofxOrbbecDevice") << "open(\"" << serial << "\") failed: " << e.getMessage();
        return false;
    }

    isOpen_ = true;
    running_ = true;
    captureThread_ = std::thread(&ofxOrbbecDevice::captureLoop, this);
    return true;
#else
    ofLogWarning("ofxOrbbecDevice")
        << "OrbbecSDK_v2 is not linked (place it under addons/ofxOrbbec/libs/OrbbecSDK/, "
           "see addon_config.mk) — open(\"" << serial << "\") is a no-op stub";
    return false;
#endif
}

void ofxOrbbecDevice::close() {
    if (running_) {
        running_ = false;
        if (captureThread_.joinable()) captureThread_.join();
    }
#ifdef OFX_ORBBEC_HAVE_SDK
    if (accelSensor_) {
        try { accelSensor_->stop(); } catch (const ob::Error&) {}
        accelSensor_.reset();
    }
    imuOk_ = false;
    if (pipeline_) {
        pipeline_->stop();
        pipeline_.reset();
    }
    align_.reset();
    device_.reset();
#endif
    isOpen_ = false;
}

void ofxOrbbecDevice::update() {
    if (frameReady_) {
        std::lock_guard<std::mutex> lock(frameMutex_);
        front_ = back_;
        front_.isNew = true;
        frameReady_ = false;
    } else {
        front_.isNew = false;
    }
}

void ofxOrbbecDevice::captureLoop() {
#ifdef OFX_ORBBEC_HAVE_SDK
    // Mirrors ofxOvrtx::renderLoop()'s "background thread writes to a
    // mutex-guarded staging buffer, update() swaps it into front_" pattern.
    while (running_) {
        std::shared_ptr<ob::FrameSet> frameSet;
        try {
            frameSet = pipeline_->waitForFrameset(100);
        } catch (const ob::Error& e) {
            ofLogWarning("ofxOrbbecDevice") << "waitForFrameset failed: " << e.getMessage();
            continue;
        }
        if (!frameSet) continue; // timeout, try again
        if (align_) {
            try {
                auto aligned = align_->process(frameSet);
                if (!aligned) continue;
                frameSet = aligned->as<ob::FrameSet>();
                if (!frameSet) continue;
            } catch (const ob::Error& e) {
                ofLogWarning("ofxOrbbecDevice") << "align failed: " << e.getMessage();
                continue;
            }
        }

        auto depthFrameRaw = frameSet->getFrame(OB_FRAME_DEPTH);
        auto colorFrameRaw = frameSet->getFrame(OB_FRAME_COLOR);
        if (!depthFrameRaw || !colorFrameRaw) continue;

        auto depthFrame = depthFrameRaw->as<ob::DepthFrame>();
        auto colorFrame = colorFrameRaw->as<ob::ColorFrame>();

        int dw = depthFrame->getWidth();
        int dh = depthFrame->getHeight();
        int cw = colorFrame->getWidth();
        int ch = colorFrame->getHeight();

        std::lock_guard<std::mutex> lock(frameMutex_);
        // OB_FORMAT_Y16 depth is already raw uint16 millimeters — matches
        // ofxOrbbecFrame::depth's ofShortPixels layout directly.
        back_.depth.setFromPixels(reinterpret_cast<const unsigned short*>(depthFrame->getData()), dw, dh, 1);
        // OB_FORMAT_RGB was requested explicitly in open(), so getData() is
        // already packed 8-bit RGB — no MJPG/format-conversion filter needed.
        back_.color.setFromPixels(colorFrame->getData(), cw, ch, OF_PIXELS_RGB);
        back_.deviceTimestampUs = depthFrame->getTimeStampUs();
        back_.frameIndex++;
        frameReady_ = true;
    }
#else
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
#endif
}
