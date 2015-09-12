#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Batch.h"
#include "cinder/gl/Context.h"
#include "cinder/Log.h"
#include "cinder/CameraUi.h"
#include "cinder/Utilities.h"

#include "CinderOculus.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace hmd;

class BasicSampleApp : public App {
public:
	BasicSampleApp();
	
	void update() override;
	void draw() override;
	void resize() override;

	void keyDown( KeyEvent event ) override;
private:
	void drawScene();
	double			mTime;

	CameraPersp			mCamera;
	CameraUi			mCameraUi;
	OculusRiftRef		mRift;

	gl::GlslProgRef	mShader;
	gl::BatchRef	mTeapot;

	vec3			mViewerPosition;
	vec4			mLightWorldPosition;
};

BasicSampleApp::BasicSampleApp()
: mViewerPosition{ vec3( 0, 0, 1 ) }
, mCameraUi( &mCamera, app::getWindow() )
{
	mShader = gl::GlslProg::create( gl::GlslProg::Format().vertex( loadAsset( "phong.vert" ) ).fragment( loadAsset( "phong.frag" ) ) );
	mTeapot = gl::Batch::create( geom::Teapot().subdivisions( 12 ), mShader );

	// Setup camera for the debug (main) window.
	mCamera.setEyePoint( vec3( 0, 2, 5 ) );
	mCamera.lookAt( vec3( 0 ) );
	mCamera.setFov( 45.0f );

	gl::disableAlphaBlending();
	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::color( Color::white() );

	try {
		mRift = OculusRift::create( OculusRift::Params().screenPercentage( 1.4f ) );
	}
	catch( const RiftExeption& exc ) {
		CI_LOG_EXCEPTION( "Failed rift initialization.", exc );
	}
}

void BasicSampleApp::update()
{
	// Animate light position.
	mTime = getElapsedSeconds();
	float t = float( mTime ) * 0.4f;
	mLightWorldPosition = vec4( math<float>::sin( t ), math<float>::sin( t * 4.0f ), math<float>::cos( t ), 1 );

	// Move head location
	if( mRift ) {
		auto host = mRift->getHostCamera();
		host.setEyePoint( mViewerPosition + vec3( 0.5f * sin( app::getElapsedSeconds() ), 0, 0 ) );
		host.lookAt( vec3( 0 ) );
		mRift->setHostCamera( host );
	}

	// Draw from update due to conflicting WM_PAINT signal emitted by ovr_submitFrame (0.7 SDK).
	gl::clear( Color( 0.02, 0.02, 0.1 ) );
	if( mRift && ! mRift->isFrameSkipped() ) {
		ScopedRiftBuffer bind{ mRift };

		for( auto eye : mRift->getEyes() ) {
			mRift->enableEye( eye );
			mShader->uniform( "uLightViewPosition", mRift->getViewMatrix() * mLightWorldPosition );
			mShader->uniform( "uSkyDirection", mRift->getViewMatrix() * vec4( 0, 1, 0, 0 ) );

			drawScene();

			// Draw positional tracking camera frustum
			CameraPersp positional;
			if( mRift->getPositionalTrackingCamera( &positional ) ) {
				gl::setModelMatrix( mat4() );
				gl::lineWidth( 1.0f );
				gl::drawFrustum( positional );
			}
		}
	}
}

void BasicSampleApp::drawScene()
{
	{
		gl::ScopedModelMatrix push;
		gl::rotate( (float)mTime, vec3( -0.3f, -1.0f, 0.2f ) );
		gl::scale( vec3( 0.5f ) );
		gl::translate( 0.0f, -0.5f, 0.0f );
		mTeapot->draw();
	}

	gl::lineWidth( 3.0f );
	gl::drawCoordinateFrame( 2 );
	gl::drawSphere( vec3( mLightWorldPosition ), 0.05f, 36 );
}

void BasicSampleApp::draw()
{
	if( ! mRift ) {
		gl::viewport( getWindowSize() );
		gl::setMatrices( mCamera );

		const mat4& view = mCamera.getViewMatrix();
		mShader->uniform( "uLightViewPosition", view * mLightWorldPosition );
		mShader->uniform( "uSkyDirection", view * vec4( 0, 1, 0, 0 ) );

		drawScene();
	}
}

void BasicSampleApp::resize()
{
	mCamera.setAspectRatio( getWindowAspectRatio() );
}

void BasicSampleApp::keyDown( KeyEvent event )
{
	switch( event.getCode() ) {
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
	case KeyEvent::KEY_t:
		mRift->enablePositionalTracking( ! mRift->isPositionalTrackingEnabled() );
		break;
	}
}

void prepareSettings( App::Settings *settings )
{
	try{
		RiftManager::initialize();
	}
	catch( const RiftExeption& exc ) {
		CI_LOG_EXCEPTION( "Failed ovr initialization", exc );
	}
	settings->setTitle( "Oculus Rift Sample" );
	settings->setWindowSize( 1920/2, 1080/2 );
}

CINDER_APP( BasicSampleApp, RendererGl( RendererGl::Options().msaa(0) ), prepareSettings );
