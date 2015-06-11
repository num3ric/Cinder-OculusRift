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
#include "cinder/Breakpoint.h"

using namespace ci;
using namespace ci::app;
using namespace hmd;

std::unique_ptr<RiftManager> RiftManager::mInstance = nullptr;
std::once_flag RiftManager::mOnceFlag;

bool OVR_VERIFY( const ovrResult& result )
{
	if( OVR_SUCCESS( result ) ) {
		return true;
	}
	else {
		ovrErrorInfo errorInfo;
		ovr_GetLastErrorInfo( &errorInfo );
		CI_LOG_E( errorInfo.ErrorString );
		CI_BREAKPOINT();
	}
	return false;
}

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
	OVR_VERIFY( ovr_Initialize( params ) );
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
, mMirrorPercentage( 0.5f )
, mHmdCaps( kDefaultHmdCaps )
, mTrackingCaps( kDefaulTrackingCaps )
, mHmdSettingsChanged( true )
, mIsExtended( false )
, mIsMirrrored( true )
, mIsMonoscopic( false )
, mUsePositionalTracking( true )
, mHmd( nullptr )
, mMirrorFBO( 0 )
, mMirrorTexture( nullptr )
, mIsRenderUpdating( true )
{
	mHostCamera.setEyePoint( vec3( 0 ) );
	mHostCamera.setViewDirection( vec3( 0, 0, 1 ) );
	
	if( OVR_SUCCESS( ovrHmd_Detect() ) ) {
		OVR_VERIFY( ovrHmd_Create( 0, &mHmd ) );
		//mHmdCaps = ovrHmd_GetEnabledCaps( mHmd ) | mHmdCaps;
		//ovrHmd_SetEnabledCaps( mHmd, mHmdCaps );
	}
	else {
		CI_LOG_E( "Failed to create Hmd." );
		ovrHmd_CreateDebug( ovrHmdType::ovrHmd_DK2, &mHmd );
	}
}

OculusRift::~OculusRift()
{
	detachFromWindow();

	if( mMirrorTexture ) {
		glDeleteFramebuffers( 1, &mMirrorFBO );
		ovrHmd_DestroyMirrorTexture( mHmd, (ovrTexture*)mMirrorTexture );
	}

	ovrHmd_DestroySwapTextureSet( mHmd, mRenderBuffer->TextureSet );

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

	initializeFrameBuffer();
	updateHmdSettings();

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
	
	OVR_VERIFY( ovrHmd_ConfigureTracking( mHmd, mTrackingCaps, 0 ) );
	
	mWindow = window;
	return true;
}

void OculusRift::createMirrorTexture()
{
	ivec2 ws = mMirrorPercentage * vec2( mHmd->Resolution.w, mHmd->Resolution.h );

 	OVR_VERIFY( ovrHmd_CreateMirrorTextureGL( mHmd, GL_RGBA, ws.x, ws.y, (ovrTexture**)&mMirrorTexture ) );

	glGenFramebuffers( 1, &mMirrorFBO );
	glBindFramebuffer( GL_READ_FRAMEBUFFER, mMirrorFBO );
	glFramebufferTexture2D( GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mMirrorTexture->OGL.TexId, 0 );
	glFramebufferRenderbuffer( GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0 );
	glBindFramebuffer( GL_READ_FRAMEBUFFER, 0 );
}

void OculusRift::initializeFrameBuffer()
{
	// Determine the size and create the buffer.
	ovrSizei left = ovrHmd_GetFovTextureSize( mHmd, ovrEye_Left, mHmd->DefaultEyeFov[ovrEye_Left], mScreenPercentage );
	ovrSizei right = ovrHmd_GetFovTextureSize( mHmd, ovrEye_Right, mHmd->DefaultEyeFov[ovrEye_Right], mScreenPercentage );
	ivec2 size;
	size.x = left.w + right.w;
	size.y = math<int>::max( left.h, right.h );

	if( ! mRenderBuffer || mRenderBuffer->getSize() != size ) {
		
		//Create render buffer (with depth)
		mRenderBuffer = std::unique_ptr<TextureBuffer>( new TextureBuffer( mHmd, size, 1, NULL, 1 ) );
		mDepthBuffer = std::unique_ptr<DepthBuffer>( new DepthBuffer( size, 0 ) );
		createMirrorTexture();
		for( int i = 0; i < ovrEye_Count; ++i ) {
			mEyeRenderDesc[i] = ovrHmd_GetRenderDesc( mHmd, (ovrEyeType)i, mHmd->DefaultEyeFov[i] );
		}

		mLayer.Header.Type = ovrLayerType_EyeFov;
		mLayer.Header.Flags = ovrLayerFlag_HighQuality; //opengl specific
		mLayer.Header.Flags |= ovrLayerFlag_TextureOriginAtBottomLeft;
		mLayer.ColorTexture[0] = mRenderBuffer->TextureSet;
		mLayer.ColorTexture[1] = mRenderBuffer->TextureSet;
		mLayer.Fov[0] = mEyeRenderDesc[0].Fov;
		mLayer.Fov[1] = mEyeRenderDesc[1].Fov;

		mLayer.Viewport[0].Pos.x = 0;
		mLayer.Viewport[0].Pos.y = 0;
		mLayer.Viewport[0].Size.w = size.x / 2;
		mLayer.Viewport[0].Size.h = size.y;

		mLayer.Viewport[1].Pos.x = size.x / 2;
		mLayer.Viewport[1].Pos.y = 0;
		mLayer.Viewport[1].Size.w = size.x / 2;
		mLayer.Viewport[1].Size.h = size.y;

		for( int i = 0; i < 2; ++i ) {
			auto vp = mLayer.Viewport[i];
			mLayer.Viewport[i].Pos.y = mRenderBuffer->getSize().y - vp.Size.h;
			//CI_LOG_I( toString( mRenderBuffer->getSize().y ) + " " + toString( vp.Size.h ) );
		}
	}
}

void OculusRift::bind() const
{
	if( mRenderBuffer ) {
		auto* set = mRenderBuffer->TextureSet;
		set->CurrentIndex = (set->CurrentIndex + 1) % set->TextureCount;
		mRenderBuffer->setAndClearRenderSurface( mDepthBuffer.get() );
	}
}

void OculusRift::unbind() const
{
	if( mRenderBuffer ) {
		mRenderBuffer->unsetRenderSurface();
	}
}

void OculusRift::enableEye( int eyeIndex, bool applyMatrices )
{
	mProjectionCached = mViewMatrixCached = mInverseViewMatrixCached = false;
	mEye = mHmd->EyeRenderOrder[eyeIndex];

	mEyeCamera.setOrientation( fromOvr( mEyeRenderPose[mEye].Orientation ) );
	mEyeCamera.setEyePoint( fromOvr( mEyeRenderPose[mEye].Position ) );
	unsigned int projectionModifier = ovrProjection_RightHanded | ovrProjection_ClipRangeOpenGL;
	mEyeCamera.mOvrProjection = fromOvr( ovrMatrix4f_Projection( mEyeRenderDesc[mEye].Fov, mEyeCamera.getNearClip(), mEyeCamera.getFarClip(), projectionModifier ) );

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
		mat4 orientation = hostOrientation * glm::toMat4( mEyeCamera.getOrientation() );
		vec3 up = vec3( orientation * vec4( 0, 1, 0, 0 ) );
		vec3 forward = vec3( orientation * vec4( 0, 0, -1, 0 ) );
		vec3 eye = mHostCamera.getEyePoint();
		eye += vec3( hostOrientation * vec4( mEyeCamera.getEyePoint(), 1 ) );
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
		mProjectionMatrix = mEyeCamera.getProjectionMatrix();
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
		positional->setOrientation( fromOvr( mEyeRenderPose[mEye].Orientation ) );
		positional->setEyePoint( fromOvr( mEyeRenderPose[mEye].Position ) );
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
	ovrTrackingState ts = ovrHmd_GetTrackingState( mHmd, ovr_GetTimeInSeconds() );
	bool tracked = (ts.StatusFlags & ovrStatus_PositionConnected) && (ts.StatusFlags & ovrStatus_PositionTracked);
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

void OculusRift::startDrawFn( Renderer *renderer )
{
	renderer->makeCurrentContext();

	if( mHmdSettingsChanged ) {
		updateHmdSettings();
	}

	if( isMonoscopic() ) {
		mEyeViewOffset[0].x = 0; // This value would normally be half the IPD,
		mEyeViewOffset[1].x = 0; //  received from the loaded profile. 
	}
	else {
		mEyeViewOffset[0] = mEyeRenderDesc[0].HmdToEyeViewOffset;
		mEyeViewOffset[1] = mEyeRenderDesc[1].HmdToEyeViewOffset;
	}

	//ovrFrameTiming ftiming = ovrHmd_GetFrameTiming( mHmd, 0 );
	//ovrTrackingState hmdState = ovrHmd_GetTrackingState( mHmd, ftiming.DisplayMidpointSeconds );
	//ovr_CalcEyePoses( hmdState.HeadPose.ThePose, mEyeViewOffset, mEyeRenderPose );

	ovrHmd_GetEyePoses( mHmd, 0, mEyeViewOffset, mEyeRenderPose, nullptr );

	mLayer.RenderPose[0] = mEyeRenderPose[0];
	mLayer.RenderPose[1] = mEyeRenderPose[1];
}

void OculusRift::finishDrawFn( Renderer *renderer )
{
	// Set up positional data.
	ovrViewScaleDesc viewScaleDesc;
	viewScaleDesc.HmdSpaceToWorldScaleInMeters = 1.0f;
	viewScaleDesc.HmdToEyeViewOffset[0] = mEyeViewOffset[0];
	viewScaleDesc.HmdToEyeViewOffset[1] = mEyeViewOffset[1];

	ovrLayerHeader* layers = &mLayer.Header;
	auto result = ovrHmd_SubmitFrame( mHmd, 0, &viewScaleDesc, &layers, 1 );
	OVR_VERIFY( result );
	mIsRenderUpdating = ( result == ovrSuccess );
	//TODO: handle ovrSuccess_NotVisible

	//mMirrorFbo->blitToScreen( mMirrorFbo->getBounds(), mWindow->getBounds() );
	// Blit mirror texture to back buffer
	glBindFramebuffer( GL_READ_FRAMEBUFFER, mMirrorFBO );
	glBindFramebuffer( GL_DRAW_FRAMEBUFFER, 0 );
	GLint w = mMirrorTexture->OGL.Header.TextureSize.w;
	GLint h = mMirrorTexture->OGL.Header.TextureSize.h;
	glBlitFramebuffer( 0, h, w, 0,
		0, 0, w, h,
		GL_COLOR_BUFFER_BIT, GL_NEAREST );
	glBindFramebuffer( GL_READ_FRAMEBUFFER, 0 );

	renderer->swapBuffers();

	// Do NOT advance TextureSet currentIndex - that has already been done above just before rendering.
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

 
