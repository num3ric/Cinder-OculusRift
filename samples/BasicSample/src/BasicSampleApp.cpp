#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Batch.h"
#include "cinder/gl/Context.h"
#include "cinder/Log.h"
#include "cinder/MayaCamUI.h"
#include "cinder/Utilities.h"

#include "CinderOculus.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class BasicSampleApp : public App {
public:
	BasicSampleApp();
	
	void update() override;
	void draw() override;
	void resize() override;

	void mouseDown( MouseEvent event ) override;
	void mouseDrag( MouseEvent event ) override;
	void keyDown( KeyEvent event ) override;
private:
	double			mTime;

	MayaCamUI		mMayaCam;
	hmd::OculusRift		mRift;

	gl::GlslProgRef	mShader;
	gl::BatchRef	mTeapot;

	vec3			mViewerPosition;
	vec4			mLightWorldPosition;
};

BasicSampleApp::BasicSampleApp()
: mViewerPosition{ vec3( 0, 0, 1 ) }
{
	if( mRift.attachToWindow( getWindow() ) ) {
		if( mRift.isDesktopExtended() )
			app::setFullScreen();
		else
			app::setWindowSize( mRift.getNativeWindowResolution() );
		CameraPersp host;
		host.setEyePoint( mViewerPosition );
		host.setCenterOfInterestPoint( vec3( 0 ) );
		mRift.setHostCamera( host );
		mRift.setScreenPercentage( 1.25f );
	}
	
	try {
		mShader = gl::GlslProg::create( gl::GlslProg::Format().vertex( loadAsset( "phong.vert" ) ).fragment( loadAsset( "phong.frag" ) ) );
	}
	catch( const std::exception& e ) {
		console() << e.what() << std::endl;
		quit();
	}
	mTeapot = gl::Batch::create( geom::Teapot().subdivisions( 12 ), mShader );

	// Setup camera for the debug (main) window.
	CameraPersp cam;
	cam.setEyePoint( vec3( 0, 2, 5 ) );
	cam.setCenterOfInterestPoint( vec3( 0 ) );
	cam.setFov( 45.0f );
	mMayaCam.setCurrentCam( cam );	

	// Generally preferable to enable vsync to prevent tearing in the headset display
	gl::enableVerticalSync();
	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::color( Color::white() );
}

void BasicSampleApp::update()
{
	// Animate light position.
	mTime = getElapsedSeconds();
	float t = float( mTime ) * 0.4f;
	mLightWorldPosition = vec4( 1.0f * math<float>::sin( t ), math<float>::sin( t * 4.0f ), 1.0f * math<float>::cos( t ), 1 );
}

void BasicSampleApp::draw()
{
	auto sceneDraw = [&]() {
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
	};
	
	mRift.bind();
	gl::clear( Color::gray( 0 ) );

	if( mRift.hasWindow( getWindow() ) ) {
		for( auto eye : mRift.getEyes() ) {
			mRift.enableEye( eye );
			
			mShader->uniform( "uLightViewPosition", mRift.getViewMatrix() * mLightWorldPosition );
			mShader->uniform( "uSkyDirection", mRift.getViewMatrix() * vec4( 0, 1, 0, 0 ) );
			sceneDraw();
			
			// Draw positional tracking camera frustum
			CameraPersp positional;
			if( mRift.getPositionalTrackingCamera( &positional ) ) {
				gl::ScopedModelMatrix push;
				gl::setModelMatrix( mRift.getHostCamera().getInverseViewMatrix() );
				gl::lineWidth( 1.0f );
				gl::drawFrustum( positional );
			}
		}
	} else {
		gl::viewport( getWindowSize() );
		gl::setMatrices( mMayaCam.getCamera() );
		
		const mat4& view = mMayaCam.getCamera().getViewMatrix();
		mShader->uniform( "uLightViewPosition", view * mLightWorldPosition );
		mShader->uniform( "uSkyDirection", view * vec4( 0, 1, 0, 0 ) );

		sceneDraw();
	}
	mRift.unbind();
}

void BasicSampleApp::resize()
{
	auto cam = mMayaCam.getCamera();
	cam.setAspectRatio( getWindowAspectRatio() );
	mMayaCam.setCurrentCam( cam );
}

void BasicSampleApp::mouseDown( MouseEvent event )
{
	mMayaCam.mouseDown( event.getPos() );
}

void BasicSampleApp::mouseDrag( MouseEvent event )
{
	mMayaCam.mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown() );
}

void BasicSampleApp::keyDown( KeyEvent event )
{
	switch( event.getCode() ) {
	case KeyEvent::KEY_ESCAPE:
		quit();
		break;
	case KeyEvent::KEY_r:
		mRift.recenterPose();
		break;
	case KeyEvent::KEY_m:
		mRift.enableMirrored( ! mRift.isMirrored() );
		break;
	case KeyEvent::KEY_s:
		mRift.enableMonoscopic( ! mRift.isMonoscopic() );
		break;
	case KeyEvent::KEY_t:
		mRift.enablePositionalTracking( ! mRift.isTracked() );
		break;
	}
}

void prepareSettings( App::Settings *settings )
{
	hmd::RiftManager::initialize();
	
	settings->disableFrameRate();
	settings->setTitle( "Oculus Rift Sample" );
	settings->setWindowSize( 1920, 1080 );
}

CINDER_APP( BasicSampleApp, RendererGl( RendererGl::Options().msaa(0) ), prepareSettings );
