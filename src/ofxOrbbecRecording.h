// ofxOrbbecRecording.h — recording a Femto's frames to a file, and playing them back as if from the camera
#pragma once
#include "ofMain.h"
#include "ofxOrbbecTypes.h"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// A take: what a skeleton tracker needs from a camera, frame by frame. The colour picture, the depth registered to
// it, and the accelerometer's up vector, with the time each frame came. One file (.femto):
//
//     "FEMTOREC1\n", uint32 n, n bytes of JSON (serial, the colour intrinsics, sizes)
//     then for each frame: double t (s since the first frame), uint8 has up, float up[3],
//         uint32 n + the colour as JPEG, uint32 width, uint32 height, uint32 n + the depth (uint16 mm) zlib-compressed
//
// The colour is JPEG (lossy, small); the depth is kept exactly.
namespace ofxOrbbecRecording {
inline const char* kMagic = "FEMTOREC1\n";
}

//--------------------------------------------------------------
// Writes frames to a .femto file. add() only copies the frame into a queue; the compressing and writing are done on
// a thread of its own, so the camera's thread is never held up. When it cannot keep up the newest frames are passed
// over (and counted).
class ofxOrbbecRecorder {
public:
    ~ofxOrbbecRecorder() { stop(); }

    bool start( const std::string& aPath, const std::string& aSerial, const ofxOrbbecIntrinsics& aColorK );
    void stop();
    bool isRecording() const { return mRunning; }

    void add( const ofPixels& aColor, const ofShortPixels& aDepth, bool aHasUp, const glm::vec3& aUp );

    size_t getFramesWritten() const { return mWritten; }
    size_t getFramesDropped() const { return mDropped; }
    uint64_t getBytesWritten() const { return mBytes; }
    double getSeconds() const { return mSeconds; }
    const std::string& getPath() const { return mPath; }

    int jpegQuality = 92;       // 0..100
    size_t maxQueued = 30;      // frames waiting to be written before new ones are passed over

protected:
    struct Item {
        ofPixels color;
        ofShortPixels depth;
        bool hasUp = false;
        glm::vec3 up{ 0.f };
        double t = 0.0;
    };
    void writeLoop();

    std::string mPath;
    std::ofstream mOut;
    std::thread mThread;
    std::atomic< bool > mRunning{ false };
    std::mutex mMutex;
    std::condition_variable mCv;
    std::deque< Item > mQueue;
    uint64_t mStartUs = 0;
    std::atomic< size_t > mWritten{ 0 }, mDropped{ 0 };
    std::atomic< uint64_t > mBytes{ 0 };
    std::atomic< double > mSeconds{ 0.0 };
};

//--------------------------------------------------------------
// Plays a .femto file back the way ofxOrbbecDevice delivers a live camera: update() then getFrame(), getUp(),
// getColorIntrinsics(). Frames come at their recorded times (times the speed), decoded on a thread of its own.
class ofxOrbbecPlayer {
public:
    ~ofxOrbbecPlayer() { close(); }

    bool open( const std::string& aPath );
    void close();
    bool isOpen() const { return mRunning; }

    // as ofxOrbbecDevice: swaps in the newest decoded frame, if there is one
    void update();
    const ofxOrbbecFrame& getFrame() const { return mFront; }
    const ofxOrbbecIntrinsics& getColorIntrinsics() const { return mK; }
    bool getUp( glm::vec3& aUp ) const;
    bool hasImu() const { return mHasUp; }
    const std::string& getSerial() const { return mSerial; }

    std::atomic< bool > loop{ true };
    std::atomic< float > speed{ 1.f };

    size_t getNumFrames() const { return mIndex.size(); }
    double getLength() const { return mIndex.empty() ? 0.0 : mIndex.back().t; }
    double getPosition() const { return mPosition; }
    bool isFinished() const { return mFinished; }

protected:
    struct Entry {
        std::streamoff offset;
        double t;
    };
    void playLoop();
    bool readFrame( size_t aIndex, ofxOrbbecFrame& aFrame, bool& aHasUp, glm::vec3& aUp );

    std::string mPath, mSerial;
    std::ifstream mIn;
    std::vector< Entry > mIndex;
    ofxOrbbecIntrinsics mK;
    std::thread mThread;
    std::atomic< bool > mRunning{ false }, mFinished{ false }, mHasUp{ false };
    std::atomic< double > mPosition{ 0.0 };
    std::mutex mMutex;
    ofxOrbbecFrame mBack, mFront;
    bool mBackReady = false;
    glm::vec3 mBackUp{ 0.f }, mUp{ 0.f };
    bool mBackHasUp = false, mFrontHasUp = false;
};
