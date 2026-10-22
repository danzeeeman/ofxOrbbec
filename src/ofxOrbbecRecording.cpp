#include "ofxOrbbecRecording.h"
#include <zlib.h>

namespace {
template< typename T > void put( std::ostream& o, const T& v ) { o.write( reinterpret_cast< const char* >( &v ), sizeof( T )); }
template< typename T > bool get( std::istream& i, T& v ) { return (bool)i.read( reinterpret_cast< char* >( &v ), sizeof( T )); }

ofJson intrinsicsToJson( const ofxOrbbecIntrinsics& k ) {
    return { { "width", k.width }, { "height", k.height }, { "fx", k.fx }, { "fy", k.fy }, { "cx", k.cx }, { "cy", k.cy },
             { "k1", k.k1 }, { "k2", k.k2 }, { "k3", k.k3 }, { "p1", k.p1 }, { "p2", k.p2 } };
}
ofxOrbbecIntrinsics intrinsicsFromJson( const ofJson& j ) {
    ofxOrbbecIntrinsics k;
    k.width = j.value( "width", 0 ); k.height = j.value( "height", 0 );
    k.fx = j.value( "fx", 0.f ); k.fy = j.value( "fy", 0.f ); k.cx = j.value( "cx", 0.f ); k.cy = j.value( "cy", 0.f );
    k.k1 = j.value( "k1", 0.f ); k.k2 = j.value( "k2", 0.f ); k.k3 = j.value( "k3", 0.f ); k.p1 = j.value( "p1", 0.f ); k.p2 = j.value( "p2", 0.f );
    return k;
}
}

//==============================================================
bool ofxOrbbecRecorder::start( const std::string& aPath, const std::string& aSerial, const ofxOrbbecIntrinsics& aColorK ) {
    stop();
    mPath = aPath;
    ofDirectory::createDirectory( ofFilePath::getEnclosingDirectory( aPath ), false, true );
    mOut.open( aPath, std::ios::binary );
    if( !mOut ) {
        ofLogError( "ofxOrbbecRecorder" ) << "could not write " << aPath;
        return false;
    }
    const ofJson th = { { "serial", aSerial }, { "color", intrinsicsToJson( aColorK ) }, { "depthAlignedToColor", true } };
    const std::string ts = th.dump();
    mOut.write( ofxOrbbecRecording::kMagic, std::strlen( ofxOrbbecRecording::kMagic ));
    put( mOut, (uint32_t)ts.size() );
    mOut.write( ts.data(), ts.size() );
    mWritten = 0; mDropped = 0; mBytes = 0; mSeconds = 0.0;
    mStartUs = 0;
    mRunning = true;
    mThread = std::thread( &ofxOrbbecRecorder::writeLoop, this );
    ofLogNotice( "ofxOrbbecRecorder" ) << "recording to " << aPath;
    return true;
}

void ofxOrbbecRecorder::stop() {
    if( !mRunning ) return;
    mRunning = false;
    mCv.notify_one();
    if( mThread.joinable() ) mThread.join();
    mOut.close();
    ofLogNotice( "ofxOrbbecRecorder" ) << mWritten << " frames (" << mDropped << " passed over), " << mBytes / ( 1024 * 1024 ) << " MB, "
                                      << ofToString( mSeconds.load(), 1 ) << " s in " << mPath;
}

void ofxOrbbecRecorder::add( const ofPixels& aColor, const ofShortPixels& aDepth, bool aHasUp, const glm::vec3& aUp ) {
    if( !mRunning ) return;
    const uint64_t tnow = ofGetElapsedTimeMicros();
    std::lock_guard< std::mutex > tlk( mMutex );
    if( mStartUs == 0 ) mStartUs = tnow;
    if( mQueue.size() >= maxQueued ) {
        mDropped++;
        return;
    }
    Item ti;
    ti.color = aColor;
    ti.depth = aDepth;
    ti.hasUp = aHasUp;
    ti.up = aUp;
    ti.t = ( tnow - mStartUs ) * 1e-6;
    mQueue.push_back( std::move( ti ));
    mCv.notify_one();
}

// The writer: each queued frame compressed and appended; what is left in the queue when stopped is still written.
void ofxOrbbecRecorder::writeLoop() {
    std::vector< Bytef > tz;
    for( ;; ) {
        Item ti;
        {
            std::unique_lock< std::mutex > tlk( mMutex );
            mCv.wait( tlk, [&] { return !mQueue.empty() || !mRunning; } );
            if( mQueue.empty() ) break;   // stopped, and everything written
            ti = std::move( mQueue.front() );
            mQueue.pop_front();
        }
        ofBuffer tjpeg;
        const ofImageQualityType tq = jpegQuality >= 95 ? OF_IMAGE_QUALITY_BEST : jpegQuality >= 85 ? OF_IMAGE_QUALITY_HIGH : OF_IMAGE_QUALITY_MEDIUM;
        if( !ofSaveImage( ti.color, tjpeg, OF_IMAGE_FORMAT_JPEG, tq )) continue;
        const uLong traw = (uLong)( ti.depth.getWidth() * ti.depth.getHeight() * sizeof( uint16_t ));
        uLongf tzlen = compressBound( traw );
        tz.resize( tzlen );
        if( compress2( tz.data(), &tzlen, reinterpret_cast< const Bytef* >( ti.depth.getData() ), traw, 1 ) != Z_OK ) continue;

        put( mOut, ti.t );
        put( mOut, (uint8_t)( ti.hasUp ? 1 : 0 ));
        put( mOut, ti.up.x ); put( mOut, ti.up.y ); put( mOut, ti.up.z );
        put( mOut, (uint32_t)tjpeg.size() );
        mOut.write( tjpeg.getData(), tjpeg.size() );
        put( mOut, (uint32_t)ti.depth.getWidth() );
        put( mOut, (uint32_t)ti.depth.getHeight() );
        put( mOut, (uint32_t)tzlen );
        mOut.write( reinterpret_cast< const char* >( tz.data() ), tzlen );
        mWritten++;
        mBytes += 8 + 1 + 12 + 4 + tjpeg.size() + 12 + tzlen;
        mSeconds = ti.t;
    }
    mOut.flush();
}

//==============================================================
bool ofxOrbbecPlayer::open( const std::string& aPath ) {
    close();
    mIn.open( aPath, std::ios::binary );
    const size_t tml = std::strlen( ofxOrbbecRecording::kMagic );
    std::string tmagic( tml, '\0' );
    uint32_t thl = 0;
    if( !mIn || !mIn.read( &tmagic[0], tml ) || tmagic != ofxOrbbecRecording::kMagic || !get( mIn, thl ) || thl > 1000000 ) {
        ofLogError( "ofxOrbbecPlayer" ) << aPath << " is not a Femto recording";
        mIn.close();
        return false;
    }
    std::string ths( thl, '\0' );
    mIn.read( &ths[0], thl );
    const ofJson th = ofJson::parse( ths, nullptr, false );
    if( th.is_discarded() ) return false;
    mSerial = th.value( "serial", std::string( "recording" ));
    mK = intrinsicsFromJson( th.value( "color", ofJson::object() ));

    // where each frame is: a pass over the lengths
    mIndex.clear();
    for( ;; ) {
        const std::streamoff toff = mIn.tellg();
        double t; uint8_t thas; float tu[3]; uint32_t tn, tw, th2, tdn;
        if( !get( mIn, t ) || !get( mIn, thas ) || !get( mIn, tu[0] ) || !get( mIn, tu[1] ) || !get( mIn, tu[2] ) || !get( mIn, tn )) break;
        mIn.seekg( tn, std::ios::cur );
        if( !get( mIn, tw ) || !get( mIn, th2 ) || !get( mIn, tdn )) break;
        mIn.seekg( tdn, std::ios::cur );
        if( !mIn ) break;   // a frame cut off at the end
        mIndex.push_back( { toff, t } );
    }
    mIn.clear();
    if( mIndex.empty() ) {
        ofLogError( "ofxOrbbecPlayer" ) << aPath << " has no frames";
        mIn.close();
        return false;
    }
    mPath = aPath;
    mFinished = false;
    mPosition = 0.0;
    mRunning = true;
    mThread = std::thread( &ofxOrbbecPlayer::playLoop, this );
    ofLogNotice( "ofxOrbbecPlayer" ) << "playing " << aPath << ": " << mIndex.size() << " frames, " << ofToString( getLength(), 1 ) << " s, "
                                    << mK.width << "x" << mK.height << " (" << mSerial << ")";
    return true;
}

void ofxOrbbecPlayer::close() {
    mRunning = false;
    if( mThread.joinable() ) mThread.join();
    if( mIn.is_open() ) mIn.close();
}

void ofxOrbbecPlayer::update() {
    std::lock_guard< std::mutex > tlk( mMutex );
    if( mBackReady ) {
        std::swap( mFront, mBack );
        mFront.isNew = true;
        mFrontHasUp = mBackHasUp;
        mUp = mBackUp;
        mHasUp = mFrontHasUp;
        mBackReady = false;
    } else {
        mFront.isNew = false;
    }
}

bool ofxOrbbecPlayer::getUp( glm::vec3& aUp ) const {
    if( !mFrontHasUp ) return false;
    aUp = mUp;
    return true;
}

bool ofxOrbbecPlayer::readFrame( size_t aIndex, ofxOrbbecFrame& aFrame, bool& aHasUp, glm::vec3& aUp ) {
    mIn.clear();
    mIn.seekg( mIndex[aIndex].offset );
    double t; uint8_t thas; uint32_t tn, tw, th, tdn;
    if( !get( mIn, t ) || !get( mIn, thas ) || !get( mIn, aUp.x ) || !get( mIn, aUp.y ) || !get( mIn, aUp.z ) || !get( mIn, tn )) return false;
    aHasUp = thas != 0;
    ofBuffer tjpeg;
    tjpeg.allocate( tn );
    if( !mIn.read( tjpeg.getData(), tn )) return false;
    if( !get( mIn, tw ) || !get( mIn, th ) || !get( mIn, tdn )) return false;
    std::vector< Bytef > tz( tdn );
    if( !mIn.read( reinterpret_cast< char* >( tz.data() ), tdn )) return false;
    if( !ofLoadImage( aFrame.color, tjpeg )) return false;
    aFrame.depth.allocate( tw, th, 1 );
    uLongf traw = (uLongf)( (size_t)tw * th * sizeof( uint16_t ));
    if( uncompress( reinterpret_cast< Bytef* >( aFrame.depth.getData() ), &traw, tz.data(), tdn ) != Z_OK ) return false;
    aFrame.deviceTimestampUs = (uint64_t)( t * 1e6 );
    aFrame.frameIndex = aIndex;
    return true;
}

// The player's thread: each frame decoded and handed over at its time in the take (times the speed), looping.
void ofxOrbbecPlayer::playLoop() {
    while( mRunning ) {
        const uint64_t tstartUs = ofGetElapsedTimeMicros();
        double tbase = 0.0;            // take time at the anchor
        uint64_t tanchorUs = tstartUs;
        float tanchorSpeed = speed;
        for( size_t i = 0; i < mIndex.size() && mRunning; i++ ) {
            ofxOrbbecFrame tf;
            bool thas = false;
            glm::vec3 tup;
            if( !readFrame( i, tf, thas, tup )) continue;
            // wait for its time; a change of speed re-anchors the clock so the position does not jump
            for( ;; ) {
                if( !mRunning ) break;
                const float ts = std::max( 0.05f, speed.load() );
                if( ts != tanchorSpeed ) {
                    tbase += ( ofGetElapsedTimeMicros() - tanchorUs ) * 1e-6 * tanchorSpeed;
                    tanchorUs = ofGetElapsedTimeMicros();
                    tanchorSpeed = ts;
                }
                const double tnow = tbase + ( ofGetElapsedTimeMicros() - tanchorUs ) * 1e-6 * tanchorSpeed;
                const double twait = ( mIndex[i].t - tnow ) / tanchorSpeed;
                if( twait <= 0.0 ) break;
                std::this_thread::sleep_for( std::chrono::microseconds( (int64_t)( std::min( twait, 0.005 ) * 1e6 )));
            }
            if( !mRunning ) break;
            {
                std::lock_guard< std::mutex > tlk( mMutex );
                std::swap( mBack, tf );
                mBackHasUp = thas;
                mBackUp = tup;
                mBackReady = true;
            }
            mPosition = mIndex[i].t;
        }
        if( !loop ) {
            mFinished = true;
            while( mRunning && !loop ) std::this_thread::sleep_for( std::chrono::milliseconds( 20 ));
            mFinished = false;
        }
    }
}
