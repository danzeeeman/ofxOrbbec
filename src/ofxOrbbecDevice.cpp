#include "ofxOrbbecDevice.h"

bool ofxOrbbecDevice::open(const std::string& serial, SyncMode syncMode) {
    if (isOpen_) {
        ofLogWarning("ofxOrbbecDevice") << "already open (" << serial_ << ")";
        return false;
    }

    serial_ = serial;
    syncMode_ = syncMode;

#ifdef OFX_ORBBEC_HAVE_SDK
    try {
        ob::Context context;
        auto deviceList = context.queryDeviceList();
        device_ = deviceList->getDeviceBySN(serial.c_str());
        if (!device_) {
            ofLogError("ofxOrbbecDevice") << "device not found: " << serial;
            return false;
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
        config->enableVideoStream(OB_STREAM_DEPTH, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_Y16);
        config->enableVideoStream(OB_STREAM_COLOR, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_RGB);
        pipeline_->start(config);

        // One synchronous frameset to read back the active stream profiles'
        // calibration (intrinsics/distortion aren't queryable before streaming
        // starts in the examples this was verified against).
        auto frameSet = pipeline_->waitForFrameset(3000);
        if (!frameSet) {
            ofLogError("ofxOrbbecDevice") << "timed out waiting for first frameset: " << serial;
            pipeline_->stop();
            return false;
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
        depthIntrinsics_ = readIntrinsics(frameSet->getFrame(OB_FRAME_DEPTH));
        colorIntrinsics_ = readIntrinsics(frameSet->getFrame(OB_FRAME_COLOR));
    } catch (const ob::Error& e) {
        ofLogError("ofxOrbbecDevice") << "open(\"" << serial << "\") failed: " << e.getMessage();
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
    if (pipeline_) {
        pipeline_->stop();
        pipeline_.reset();
    }
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
            ofLogError("ofxOrbbecDevice") << "waitForFrameset failed: " << e.getMessage();
            continue;
        }
        if (!frameSet) continue; // timeout, try again

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
