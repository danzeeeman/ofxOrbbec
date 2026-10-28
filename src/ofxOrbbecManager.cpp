#include "ofxOrbbecManager.h"

std::vector<std::string> ofxOrbbecManager::listDevices() {
#ifdef OFX_ORBBEC_HAVE_SDK
    try {
        ob::Context context;
        auto deviceList = context.queryDeviceList();
        std::vector<std::string> serials;
        serials.reserve(deviceList->getCount());
        for (uint32_t i = 0; i < deviceList->getCount(); ++i) {
            serials.emplace_back(deviceList->getSerialNumber(i));
        }
        return serials;
    } catch (const ob::Error& e) {
        ofLogWarning("ofxOrbbecManager") << "listDevices() failed: " << e.getMessage();
        return {};
    }
#else
    ofLogVerbose("ofxOrbbecManager")
        << "OrbbecSDK_v2 not linked — listDevices() always returns empty until it is";
    return {};
#endif
}

bool ofxOrbbecManager::openAll(const std::vector<std::string>& serials) {
    closeAll();

    if (serials.empty()) {
        ofLogWarning("ofxOrbbecManager") << "openAll() called with no device serials";
        return false;
    }

    bool allOk = true;
    for (size_t i = 0; i < serials.size(); ++i) {
        auto dev = std::make_unique<ofxOrbbecDevice>();
        auto syncMode = (i == 0) ? ofxOrbbecDevice::SyncMode::Master
                                  : ofxOrbbecDevice::SyncMode::Subordinate;
        allOk &= dev->open(serials[i], syncMode);
        devices_.push_back(std::move(dev));
    }
    return allOk;
}

void ofxOrbbecManager::closeAll() {
    for (auto& dev : devices_) dev->close();
    devices_.clear();
}

void ofxOrbbecManager::update() {
    for (auto& dev : devices_) dev->update();
}
