#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Log.h"
#include "cinder/Utilities.h"

#include "CinderOculus.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class InstancedStereoApp : public App {
public:
	InstancedStereoApp();

	void update() override;
	void draw() override;
	void keyDown( KeyEvent event ) override;
private:
	void drawPositionalTrackingCamera() const;
	double			mTime;

	hmd::OculusRiftRef	mRift;

	gl::GlslProgRef	mShader;
	gl::BatchRef	mTeapot;

	vec4			mLightWorldPosition;
};

InstancedStereoApp::InstancedStereoApp()
: mLightWorldPosition( vec4( 1, 1, 1, 1 ) )
{
	CameraPersp host;
	host.setEyePoint( vec3( 0, 0, 1 ) );
	host.lookAt( vec3( 0 ) );
	mRift = hmd::OculusRift::create( hmd::OculusRift::Params().hostCamera( host ).screenPercentage( 1.25f ) );

	try {
		mShader = gl::GlslProg::create( gl::GlslProg::Format().vertex( loadAsset( "phong.vert" ) ).fragment( loadAsset( "phong.frag" ) ) );
	}
	catch( const std::exception& e ) {
		console() << e.what() << std::endl;
		quit();
	}
	mTeapot = gl::Batch::create( geom::Teapot().subdivisions( 12 ), mShader );

	gl::enableVerticalSync( false );
	gl::disableAlphaBlending();
	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::enable( GL_CLIP_DISTANCE0, true );
}

void InstancedStereoApp::update()
{
	mTime = getElapsedSeconds();
	gl::clear();

	hmd::ScopedRiftBuffer bind{ mRift };
	std::array<mat4, 6> worldToEyeClipMatrices;

	// Calc clip space conversion matrices for both eyes
	for( auto eye : mRift->getEyes() ) {
		gl::ScopedMatrices push;
		mRift->enableEye( eye );
		auto idx = 3 * static_cast<size_t>(eye);
		worldToEyeClipMatrices.at( idx ) = mRift->getViewMatrix() * mRift->getModelMatrix();
		worldToEyeClipMatrices.at( idx + 1 ) = mRift->getProjectionMatrix();
		
		// non-instanced scene
		gl::lineWidth( 3.0f );
		gl::drawCoordinateFrame( 2 );
		gl::drawSphere( vec3( mLightWorldPosition ), 0.05f, 36 );
	}

	{
		gl::ScopedViewport port{ vec2( 0 ), mRift->getFboSize() };
		gl::ScopedModelMatrix push;
		gl::rotate( (float)mTime, vec3( -0.3f, -1.0f, 0.2f ) );
		gl::scale( vec3( 0.5f ) );
		gl::translate( 0.0f, -0.5f, 0.0f );

		auto normalMatrix = glm::transpose( glm::inverse( gl::getModelMatrix() ) );
		worldToEyeClipMatrices.at( 2 ) = worldToEyeClipMatrices.at( 5 ) = normalMatrix;
		mShader->uniform( "uLightPosition", mLightWorldPosition );
		mShader->uniform( "uWorldToEyeClipMatrices", worldToEyeClipMatrices.data(), 6 );
		mTeapot->drawInstanced( 2 );
	}
}

void InstancedStereoApp::draw()
{

}

void InstancedStereoApp::keyDown( KeyEvent event )
{
	hideCursor();
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
		mRift->enablePositionalTracking( ! mRift->isTracked() );
		break;
	}
}

void prepareSettings( App::Settings *settings )
{
	hmd::RiftManager::initialize();

	settings->disableFrameRate();
	settings->setTitle( "Oculus Rift Sample" );
	settings->setWindowSize( 1920/2, 1080/2 );
}

CINDER_APP( InstancedStereoApp, RendererGl( RendererGl::Options().msaa( 0 ) ), prepareSettings );