// ofxOrbbecManager.h — N-device discovery + hardware-synced capture
#pragma once
#include "ofxOrbbecDevice.h"
#include <vector>
#include <memory>
#include <string>

// Owns N ofxOrbbecDevice instances, discovers connected Femto Megas, and
// coordinates hardware trigger-cable sync (first device opened as sync
// Master, the rest as Subordinate).
class ofxOrbbecManager {
public:
    // Discovers all connected Orbbec devices. Returns their serial numbers.
    static std::vector<std::string> listDevices();

    // Opens every serial in `serials` (or every connected device if empty),
    // wiring the first as sync Master and the rest as Subordinate over the
    // trigger cable. For a rig with no trigger cable, open devices one at a
    // time via device-level open() with SyncMode::Standalone instead.
    bool openAll(const std::vector<std::string>& serials = {});
    void closeAll();

    // Calls update() on every owned device. Call once per frame.
    void update();

    size_t deviceCount() const { return devices_.size(); }
    ofxOrbbecDevice&       device(size_t index)       { return *devices_.at(index); }
    const ofxOrbbecDevice& device(size_t index) const { return *devices_.at(index); }

private:
    std::vector<std::unique_ptr<ofxOrbbecDevice>> devices_;
};
