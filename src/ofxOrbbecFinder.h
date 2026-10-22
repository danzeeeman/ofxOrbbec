// ofxOrbbecFinder.h — the Orbbec cameras there are: on USB, and on the network (Ethernet / PoE)
#pragma once
#include "ofMain.h"
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Looks for cameras on a thread of its own, every few seconds: those on USB, and those on the network, found by the
// SDK's network discovery (which needs the computer on the cameras' subnet, and, on macOS, the app allowed on the
// local network). getFound() is safe from any thread.
class ofxOrbbecFinder {
public:
    struct Found {
        std::string serial, ip, name, connection;   // ip empty for a camera on USB
    };

    ~ofxOrbbecFinder() { stop(); }

    void start( float aEverySec = 3.f );
    void stop();

    std::vector< Found > getFound() const;
    // seconds since the last search finished (large before the first)
    float sinceSearch() const;

protected:
    void loop();

    std::thread mThread;
    std::atomic< bool > mRunning{ false };
    float mEverySec = 3.f;
    mutable std::mutex mMutex;
    std::vector< Found > mFound;
    uint64_t mLastSearchUs = 0;
};
