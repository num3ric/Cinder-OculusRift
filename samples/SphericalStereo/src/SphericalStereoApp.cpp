#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Log.h"
#include "cinder/gl/VboMesh.h"
#include "cinder/Utilities.h"
#include "cinder/GeomIo.h"
#include "cinder/gl/Shader.h"
#include "cinder/Text.h"

#include "CinderOculus.h"

#include <algorithm>

using namespace ci;
using namespace ci::app;
using namespace std;

// Spherical latlong file names should follow the Gear VR convention: _LR, _RL, _TB, _BT.
struct Pano {
	Pano() { }
	Pano( const fs::path& path )
		: mDisplayMode{ ivec2(0,0) }, mName( path.filename().string() )
	{
		std::string stem = path.stem().string();
		std::transform( stem.begin(), stem.end(), stem.begin(), ::toupper );
		switch( stem.back() ) {
		case 'L': mDisplayMode = ivec2( 0, 0 ); break;
		case 'R': mDisplayMode = ivec2( 0, 1 ); break;
		case 'T': mDisplayMode = ivec2( 1, 0 ); break;
		case 'B': mDisplayMode = ivec2( 1, 1 ); break;
		default: break;
		}

		mLatLong = gl::Texture2d::create( loadImage( path ), gl::Texture2d::Format().wrap( GL_REPEAT ) );
	}
	gl::Texture2dRef	mLatLong;
	std::string			mName;
	ivec2				mDisplayMode;
};

class SphericalStereoApp : public App {
public:
	SphericalStereoApp();
	void update() override;
	void draw() override;

	void fileDrop( FileDropEvent event ) override;
	void keyDown( KeyEvent event ) override;
	void keyUp( KeyEvent event ) override;

	hmd::OculusRiftRef	mRift;
private:
	
	size_t				mPanoIndex = 0;
	std::vector<Pano>	mPanos;
	gl::BatchRef		mSphere;
	gl::GlslProgRef		mStereoGlsl;
};

SphericalStereoApp::SphericalStereoApp()
{
	mRift = hmd::OculusRift::create( hmd::OculusRift::Params()
		.monoscopic( true )
		.positional( false ) );

	try{
		mStereoGlsl = gl::GlslProg::create( loadAsset( "stereo.vert" ), loadAsset( "stereo.frag" ) );
	}
	catch( const ci::Exception& exc ) {
		CI_LOG_EXCEPTION( "GLSL", exc );
	}
	mPanos.push_back( Pano{ getAssetPath( "arnold_LR.jpg" ) } );
	mSphere		= gl::Batch::create( geom::Icosphere().subdivisions( 2 ), mStereoGlsl );

	// Generally preferable to enable vsync to prevent tearing in the headset display
	gl::enableVerticalSync( false );
	gl::color( Color::white() );
}

void SphericalStereoApp::update()
{
	if( mRift ) {
		hmd::ScopedRiftBuffer bind{ mRift };
		gl::clear();
		for( auto eye : mRift->getEyes() ) {
			mRift->enableEye( eye );
			const auto& pano = mPanos.at( mPanoIndex );
			mStereoGlsl->uniform( "uDisplayMode", pano.mDisplayMode );
			mStereoGlsl->uniform( "uRightEye", static_cast<int>(eye) );
			gl::ScopedTextureBind tex0( pano.mLatLong );
			mSphere->draw();

			{
				auto size = mRift->getFboSize();
				gl::ScopedMatrices push;
				gl::setMatricesWindow( size.x / 2, size.y );
				vec3 latencies = mRift->getLatencies();

				stringstream ss;
				ss << mPanos.at( mPanoIndex ).mName << std::endl;
				ss << " " << std::endl;
				ss << "App fps: " << toString( getAverageFps() ) << std::endl;
				ss << "Ren: " << latencies.x << std::endl;
				ss << "TWrp: " << latencies.y << std::endl;
				ss << "PostPresent: " << latencies.z << std::endl;

				auto tbox = TextBox().text( ss.str() ).font( Font( "Arial", 20.0f ) ).color( Color::white() ).backgroundColor( Color::black() );
				gl::draw( gl::Texture2d::create( tbox.render() ), vec2( size.x / 3, size.y / 2 ) );
			}
		}
	}
}

void SphericalStereoApp::draw()
{
	
}

void SphericalStereoApp::fileDrop( FileDropEvent event )
{
	mPanos.insert( mPanos.begin() + mPanoIndex, Pano{ event.getFile( 0 ) } );
}

void SphericalStereoApp::keyDown( KeyEvent event )
{
	hideCursor();
	switch( event.getCode() ) {
	case KeyEvent::KEY_SPACE:
		mPanoIndex = ( mPanoIndex + 1 ) % mPanos.size();
		break;
	case KeyEvent::KEY_ESCAPE:
		quit();
		break;
	case KeyEvent::KEY_r:
		mRift->recenterPose();
		break;
	case KeyEvent::KEY_m:
		mRift->enableMirrored( ! mRift->isMirrored() );
		break;
	case KeyEvent::KEY_s:
		mRift->enableMonoscopic( ! mRift->isMonoscopic() );
		break;
	case KeyEvent::KEY_p:
		mRift->enablePositionalTracking( ! mRift->isPositionalTrackingEnabled() );
		break;
	}
}

void SphericalStereoApp::keyUp( KeyEvent event )
{

}

CINDER_APP( SphericalStereoApp, RendererGl( RendererGl::Options().msaa( 0 ) ), []( App::Settings *settings ) {
	hmd::RiftManager::initialize();
	settings->disableFrameRate();
	settings->setTitle( "Oculus Rift Sample" );
	settings->setWindowSize( 1920/2, 1080/2 );
} )
