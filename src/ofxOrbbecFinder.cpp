#include "ofxOrbbecFinder.h"
#ifdef OFX_ORBBEC_HAVE_SDK
#include <libobsensor/ObSensor.hpp>
#endif

void ofxOrbbecFinder::start( float aEverySec ) {
    if( mRunning ) return;
    mEverySec = std::max( 0.5f, aEverySec );
    mRunning = true;
    mThread = std::thread( &ofxOrbbecFinder::loop, this );
}

void ofxOrbbecFinder::stop() {
    mRunning = false;
    if( mThread.joinable() ) mThread.join();
}

std::vector< ofxOrbbecFinder::Found > ofxOrbbecFinder::getFound() const {
    std::lock_guard< std::mutex > tlk( mMutex );
    return mFound;
}

float ofxOrbbecFinder::sinceSearch() const {
    std::lock_guard< std::mutex > tlk( mMutex );
    return mLastSearchUs ? ( ofGetElapsedTimeMicros() - mLastSearchUs ) * 1e-6f : 1e9f;
}

// One context for the whole time: with network discovery on, the SDK keeps finding cameras in the background and
// queryDeviceList() returns what it has found.
void ofxOrbbecFinder::loop() {
#ifdef OFX_ORBBEC_HAVE_SDK
    std::unique_ptr< ob::Context > tctx;
    try {
        tctx = std::make_unique< ob::Context >();
        tctx->enableNetDeviceEnumeration( true );
    } catch( const ob::Error& e ) {
        ofLogError( "ofxOrbbecFinder" ) << "could not start looking for cameras: " << e.getMessage();
        mRunning = false;
        return;
    }
    while( mRunning ) {
        std::vector< Found > tfound;
        try {
            auto tlist = tctx->queryDeviceList();
            for( uint32_t i = 0; tlist && i < tlist->getCount(); i++ ) {
                Found f;
                f.serial = tlist->getSerialNumber( i ) ? tlist->getSerialNumber( i ) : "";
                f.name = tlist->getName( i ) ? tlist->getName( i ) : "";
                f.connection = tlist->getConnectionType( i ) ? tlist->getConnectionType( i ) : "";
                const char* tip = tlist->getIpAddress( i );
                f.ip = tip ? tip : "";
                if( f.ip == "0.0.0.0" ) f.ip.clear();
                tfound.push_back( f );
            }
        } catch( const ob::Error& e ) {
            ofLogWarning( "ofxOrbbecFinder" ) << "looking for cameras: " << e.getMessage();
        }
        {
            std::lock_guard< std::mutex > tlk( mMutex );
            mFound = std::move( tfound );
            mLastSearchUs = ofGetElapsedTimeMicros();
        }
        for( float t = 0.f; t < mEverySec && mRunning; t += 0.05f ) std::this_thread::sleep_for( std::chrono::milliseconds( 50 ));
    }
#else
    ofLogWarning( "ofxOrbbecFinder" ) << "OrbbecSDK_v2 is not linked: no cameras to find";
    mRunning = false;
#endif
}
