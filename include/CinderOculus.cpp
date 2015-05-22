/*
 *
 * Copyright (c) 2015
 * Paul Houx (The Barbarian Group)
 * Eric Renaud-Houde (Moving Picture Company)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of the Paul Houx/Eric Renaud-Houde nor the names of its
 * contributors may be used to endorse or promote products
 * derived from this software without specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "CinderOculus.h"
#include "cinder/Log.h"
#include "cinder/gl/VboMesh.h"
#include "cinder/Utilities.h"

//#include "OVR_Kernel.h"
//#include "Kernel/OVR_Threads.h"

using namespace ci;
using namespace ci::app;
using namespace hmd;

std::unique_ptr<RiftManager> RiftManager::mInstance = nullptr;
std::once_flag RiftManager::mOnceFlag;

void RiftManager::initialize()
{
	std::call_once( mOnceFlag,
		[] {
		mInstance.reset( new RiftManager );
	} );
}

RiftManager::RiftManager()
{
	ovrInitParams * params = nullptr; // default options
	ovrBool result = ovr_Initialize( params );
	if( ! result ) {
		throw std::runtime_error( "Failed to initialize Oculus VR." );
	}
}

RiftManager::~RiftManager()
{
	ovr_Shutdown();
}

static const unsigned int kDefaultHmdCaps =
  ovrHmdCap_LowPersistence
| ovrHmdCap_DynamicPrediction;

static const unsigned int kDefaulTrackingCaps =
  ovrTrackingCap_Orientation
| ovrTrackingCap_MagYawCorrection
| ovrTrackingCap_Position;

OculusRift::OculusRift()
: mWindow( nullptr )
, mHeadScale( 1.0f )
, mScreenPercentage( 1.0f )
, mHmdCaps( kDefaultHmdCaps )
, mTrackingCaps( kDefaulTrackingCaps )
, mHmdSettingsChanged( true )
, mIsExtended( false )
, mIsMirrrored( true )
, mIsMonoscopic( false )
, mUsePositionalTracking( true )
, mHmd( nullptr )
{
	mHostCamera.setEyePoint( vec3( 0 ) );
	mHostCamera.setViewDirection( vec3( 0, 0, 1 ) );
	
	if( ovrHmd_Detect() > 0 ) {
		auto result = ovrHmd_Create( 0, &mHmd );
		CI_ASSERT( result );
		// Set hmd capabilities.
		mHmdCaps = ovrHmd_GetEnabledCaps( mHmd ) | mHmdCaps;
		ovrHmd_SetEnabledCaps( mHmd, mHmdCaps );
	}
	else {
		CI_LOG_E( "Failed to create Hmd." );
		ovrHmd_CreateDebug( ovrHmdType::ovrHmd_DK2, &mHmd );
	}
}

OculusRift::~OculusRift()
{
	detachFromWindow();

	if( mHmd )
		ovrHmd_Destroy( mHmd );
	
	mHmd = nullptr;
}

void OculusRift::detachFromWindow()
{
	if( isValid( mWindow ) ) {
		RendererGlRef rendererGl = std::dynamic_pointer_cast<RendererGl>( mWindow->getRenderer() );
		if( rendererGl ) {
			rendererGl->setStartDrawFn( nullptr );
			rendererGl->setFinishDrawFn( nullptr );
		}
		mWindow.reset();
	}
}

bool OculusRift::initialize( const ci::app::WindowRef& window )
{
	if( ! mHmd || ! isValid( window ) )
		return false;

	// Create or resize the frame buffer.
	initializeFrameBuffer();
	
	// Override the window's startDraw() and finishDraw() methods, so we can inject our own code.
	RendererGlRef rendererGl = std::dynamic_pointer_cast<RendererGl>( window->getRenderer() );
	if( rendererGl ) {
		rendererGl->setStartDrawFn( std::bind( &OculusRift::startDrawFn, this, std::placeholders::_1 ) );
		rendererGl->setFinishDrawFn( std::bind( &OculusRift::finishDrawFn, this, std::placeholders::_1 ) );
	}
	else {
		throw std::runtime_error( "CinderOculus can only be used in combination with RendererGl." );
	}
	// Connect to the window close event, so we can properly destroy our HMD.
	window->getSignalClose().connect( std::bind( [&]( ovrHmd hmd ) { assert( hmd == mHmd ); detachFromWindow(); }, mHmd ) );
	
	ovrHmd_ConfigureTracking( mHmd, mTrackingCaps, 0 );
	for( int i = 0; i < ovrEye_Count; ++i ) {
		mEyeRenderDesc[i] = ovrHmd_GetRenderDesc( mHmd, (ovrEyeType)i, mHmd->MaxEyeFov[i] );
	}
	
	updateHmdSettings();
	
	mWindow = window;
	return true;
}

void OculusRift::bind() const
{
	if ( mFbo ) {
		mFbo->bindFramebuffer();
	}
}

void OculusRift::unbind() const
{
	if ( mFbo ) {
		mFbo->unbindFramebuffer();
	}
}

void OculusRift::enableEye( int eyeIndex, bool applyMatrices )
{
	mProjectionCached = mViewMatrixCached = mInverseViewMatrixCached = false;
	mEye = mHmd->EyeRenderOrder[eyeIndex];

	mHmdEyeCamera.setOrientation( fromOvr( mEyeRenderPose[mEye].Orientation ) );
	mHmdEyeCamera.setEyePoint( fromOvr( mEyeRenderPose[mEye].Position ) );
	mHmdEyeCamera.mOvrProjection = fromOvr( ovrMatrix4f_Projection( mEyeRenderDesc[mEye].Fov, mHmdEyeCamera.getNearClip(), mHmdEyeCamera.getFarClip(), true ) );

	if( applyMatrices ) {
		gl::viewport( getEyeViewport() );
		gl::setModelMatrix( mat4() );
		gl::setViewMatrix( getViewMatrix() );
		gl::setProjectionMatrix( getProjectionMatrix() );
	}
}

std::list<ovrEyeType> OculusRift::getEyes() const
{
	if( mHmd && isValid( mWindow ) )
		return { mHmd->EyeRenderOrder[0], mHmd->EyeRenderOrder[1] };
	
	return {};
}

glm::mat4 OculusRift::getViewMatrix() const
{
	if( ! mViewMatrixCached ) {
		mat4 hostOrientation = glm::toMat4( mHostCamera.getOrientation() );
		mat4 orientation = hostOrientation * glm::toMat4( mHmdEyeCamera.getOrientation() );
		vec3 up = vec3( orientation * vec4( 0, 1, 0, 0 ) );
		vec3 forward = vec3( orientation * vec4( 0, 0, -1, 0 ) );
		vec3 eye = mHostCamera.getEyePoint();
		if( isTracked() ) {
			eye += vec3( hostOrientation * vec4( mHmdEyeCamera.getEyePoint(), 1 ) );
		}
		mViewMatrix = mat4( glm::lookAt( eye, eye + forward, up ) ) * glm::scale( vec3( 1.0f / mHeadScale ) );
		mViewMatrixCached = true;
	}
	return mViewMatrix;
}

glm::mat4 OculusRift::getInverseViewMatrix() const
{
	if( ! mInverseViewMatrixCached ) {
		mInverseViewMatrix = glm::inverse( getViewMatrix() );
		mInverseViewMatrixCached = true;
	}
	return mInverseViewMatrix;
}

glm::mat4 OculusRift::getProjectionMatrix() const
{
	if( ! mProjectionCached ) {
		mProjectionMatrix = mHmdEyeCamera.getProjectionMatrix();
		mProjectionCached = true;
	}
	return mProjectionMatrix;
}

//bool OculusRift::isCaptured() const
//{
//	if( mHmd )
//		return ovrHmd_GetEnabledCaps( mHmd ) & ovrHmdCap_Captured;
//	return false;
//}

bool OculusRift::isDesktopExtended() const
{
	return mIsExtended;
}

void OculusRift::recenterPose()
{
	ovrHmd_RecenterPose( mHmd );
}

bool OculusRift::getPositionalTrackingCamera( CameraPersp* positional ) const
{
	if( isTracked() ) {
		float aspectRatio = abs( tan( 0.5f * mHmd->CameraFrustumHFovInRadians ) / tan( 0.5f * mHmd->CameraFrustumVFovInRadians ) );
		positional->setPerspective(	toDegrees( mHmd->CameraFrustumVFovInRadians ),
									aspectRatio,
									- mHmd->CameraFrustumNearZInMeters,
									- mHmd->CameraFrustumFarZInMeters );
		positional->setOrientation( fromOvr( mTrackingState.CameraPose.Orientation ) );
		positional->setEyePoint( fromOvr( mTrackingState.CameraPose.Position ) );
		return true;
	}
	return false;
}


glm::vec3 OculusRift::getLatencies() const
{
	float latencies[3] = { 0.0f, 0.0f, 0.0f };
	if( ovrHmd_GetFloatArray( mHmd, "DK2Latency", latencies, 3 ) == 3 ) {
		return 1000.0f * glm::vec3( latencies[0], latencies[1], latencies[2] );
	}
	return glm::vec3( 0 );
}

void OculusRift::setScreenPercentage( float sp )
{
	CI_ASSERT( sp > 0.0f );
	mScreenPercentage = sp;
}

void OculusRift::setMirrorPercentage( float sp )
{
	CI_ASSERT( sp > 0.0f );
	mMirrorPercentage = sp;
}

bool OculusRift::isTracked() const
{
	auto status = mTrackingState.StatusFlags;
	bool tracked = ( status & ovrStatus_PositionConnected ) && ( status & ovrStatus_PositionTracked );
	return tracked && isPositionalTrackingEnabled();
}

bool OculusRift::isMirrored() const
{
	return mIsMirrrored;
}

void OculusRift::enableMirrored( bool enabled )
{
	if( mIsMirrrored != enabled ) {
		mIsMirrrored = enabled;
		mHmdSettingsChanged = true;
	}
}

void OculusRift::setHeadScale( float scale )
{
	mHeadScale = scale;
}

bool OculusRift::isValid( const WindowRef& window )
{
	return window && window->isValid();
}

void OculusRift::updateHmdSettings()
{
	ovrHmd_SetEnabledCaps( mHmd, mHmdCaps );
	mHmdSettingsChanged = false;
}

void OculusRift::initializeFrameBuffer()
{
	// Determine the size and create the buffer.
	ovrSizei left = ovrHmd_GetFovTextureSize( mHmd, ovrEye_Left, mHmd->DefaultEyeFov[ovrEye_Left], mScreenPercentage );
	ovrSizei right = ovrHmd_GetFovTextureSize( mHmd, ovrEye_Right, mHmd->DefaultEyeFov[ovrEye_Right], mScreenPercentage );
	int w = left.w + right.w;
	int h = math<int>::max( left.h, right.h );
	
	if( ! mFbo || mFbo->getWidth() != w || mFbo->getHeight() != h ) {


		// Create mirror texture and an FBO used to copy mirror texture to back buffer
		ovrGLTexture* mirrorTexture;
		ovrHmd_CreateMirrorTextureGL( mHmd, GL_RGBA, w, h, (ovrTexture**)&mirrorTexture );
		// Configure the mirror read buffer
		GLuint mirrorFBO = 0;
		gl::TextureRef mirror = gl::Texture::create( GL_TEXTURE_2D, mirrorTexture->OGL.TexId, w, h, false );
		// Specify the buffer format.
		gl::Fbo::Format fmt;
		fmt.attachment( GL_COLOR_ATTACHMENT0, mirror );
		fmt.enableDepthBuffer();
		fmt.setSamples( 0 ); // TODO: support multi-sampling
		mFbo = gl::Fbo::create( w, h, fmt )
		
		// The actual size may be different due to hardware limits.
		w = mFbo->getWidth();
		h = mFbo->getHeight();
		
		// Initialize eye rendering information.
		for( ovrEyeType eye = ovrEye_Left; eye < ovrEye_Count; eye = static_cast<ovrEyeType>( eye + 1 ) ) {
			mEyeTexture[eye].OGL.Header.API = ovrRenderAPI_OpenGL;
			mEyeTexture[eye].OGL.Header.TextureSize = toOvr( mFbo->getSize() );
			//mEyeTexture[eye].OGL.Header.RenderViewport.Size.w = mEyeViewport[eye].Size.w = w / 2;
			//mEyeTexture[eye].OGL.Header.RenderViewport.Size.h = mEyeViewport[eye].Size.h = h;
			//mEyeTexture[eye].OGL.Header.RenderViewport.Pos.x = mEyeViewport[eye].Pos.x = eye * ( w + 1 ) / 2;
			//mEyeTexture[eye].OGL.Header.RenderViewport.Pos.y = mEyeViewport[eye].Pos.y = 0;
			mEyeTexture[eye].OGL.TexId = mFbo->getColorTexture()->getId();
		}
	}
}

void OculusRift::startDrawFn( Renderer *renderer )
{
	renderer->makeCurrentContext();
	initializeFrameBuffer();

	if( mHmdSettingsChanged ) {
		updateHmdSettings();
	}

	// Update eye render poses.
	ovrVector3f hmdToEyeViewOffset[2] = { mEyeRenderDesc[0].HmdToEyeViewOffset, mEyeRenderDesc[1].HmdToEyeViewOffset };
	if( isMonoscopic() ) {
		hmdToEyeViewOffset[0].x = 0; // This value would normally be half the IPD,
		hmdToEyeViewOffset[1].x = 0; //  received from the loaded profile. 
	}
	ovrHmd_GetEyePoses( mHmd, 0, hmdToEyeViewOffset, mEyeRenderPose, &mTrackingState );
}

void OculusRift::finishDrawFn( Renderer *renderer )
{
	// Set up positional data.
	ovrViewScaleDesc viewScaleDesc;
	viewScaleDesc.HmdSpaceToWorldScaleInMeters = 1.0f;
	viewScaleDesc.HmdToEyeViewOffset[0] = ViewOffset[0];
	viewScaleDesc.HmdToEyeViewOffset[1] = ViewOffset[1];

	ovrLayerEyeFov ld;
	ld.Header.Type = ovrLayerType_EyeFov;
	ld.Header.Flags = ovrLayerFlag_TextureOriginAtBottomLeft;   // Because OpenGL.

	for( int eye = 0; eye < 2; eye++ )
	{
		ld.ColorTexture[eye] = eyeRenderTexture[eye]->TextureSet;
		ld.Viewport[eye] = Recti( eyeRenderTexture[eye]->GetSize() );
		ld.Fov[eye] = mHmd->DefaultEyeFov[eye];
		ld.RenderPose[eye] = EyeRenderPose[eye];
	}

	ovrLayerHeader* layers = &ld.Header;
	ovrResult result = ovrHmd_SubmitFrame( HMD, 0, &viewScaleDesc, &layers, 1 );
}

ScopedBind::ScopedBind( OculusRift& rift )
: mRift( &rift )
{
	mRift->bind();
}

ScopedBind::~ScopedBind()
{
	mRift->unbind();
}

 
