// ofxOrbbecTypes.h — shared frame/intrinsics types for ofxOrbbec
#pragma once
#include "ofMain.h"
#include <cstdint>

struct ofxOrbbecIntrinsics {
    int   width  = 0;
    int   height = 0;
    float fx = 0, fy = 0, cx = 0, cy = 0;
    float k1 = 0, k2 = 0, k3 = 0, p1 = 0, p2 = 0;
};

struct ofxOrbbecFrame {
    ofShortPixels depth;              // raw depth, millimeters
    ofPixels      color;              // RGB
    uint64_t      deviceTimestampUs = 0;
    uint64_t      frameIndex        = 0;
    bool          isNew             = false;
};
