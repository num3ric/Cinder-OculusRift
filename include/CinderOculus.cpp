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
#include "cinder/app/App.h"
#include "cinder/Utilities.h"
#include "cinder/Breakpoint.h"

using namespace ci;
using namespace ci::app;
using namespace hmd;

std::unique_ptr<RiftManager> RiftManager::mInstance = nullptr;
std::once_flag RiftManager::mOnceFlag;

void OVR_VERIFY( const ovrResult& result )
{
	if( ! OVR_SUCCESS( result ) ) {
		ovrErrorInfo errorInfo;
		ovr_GetLastErrorInfo( &errorInfo );
		throw RiftExeption( errorInfo.ErrorString );
	}
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
	OVR_VERIFY( ovr_Initialize( nullptr ) );
}

RiftManager::~RiftManager()
{
	ovr_Shutdown();
}

static const unsigned int kDefaulTrackingCaps =
  ovrTrackingCap_Orientation
| ovrTrackingCap_MagYawCorrection
| ovrTrackingCap_Position;


OculusRiftRef OculusRift::create( const Params& params )
{
	return OculusRiftRef( new OculusRift{ params } );
}

OculusRift::OculusRift( const Params& params )
: mScreenPercentage( params.mScreenPercentage )
, mTrackingCaps( kDefaulTrackingCaps )
, mIsMirrrored( params.mIsMirrrored )
, mIsMonoscopic( params.mIsMonoscopic )
, mUsePositionalTracking( params.mUsePositionalTracking )
, mHmd( nullptr )
, mMirrorFBO( 0 )
, mMirrorTexture( nullptr )
, mSkipFrame( false )
{
	if( params.mHostCam.first ) {
		mHostCamera = params.mHostCam.second;
	}
	else {
		mHostCamera.setEyePoint( vec3( 0 ) );
		mHostCamera.setViewDirection( vec3( 0, 0, -1 ) );
	}
	
	ovrGraphicsLuid luid; //can't use in opengl
	OVR_VERIFY( ovr_Create( &mHmd, &luid ) );
	mHmdDesc = ovr_GetHmdDesc( mHmd );


	OVR_VERIFY( ovr_ConfigureTracking( mHmd, mTrackingCaps, 0 ) );
	for( int i = 0; i < ovrEye_Count; ++i ) {
		mEyeRenderDesc[i] = ovr_GetRenderDesc( mHmd, (ovrEyeType)i, mHmdDesc.DefaultEyeFov[i] );
	}

	initializeFrameBuffer();
	initializeMirrorTexture( app::getWindowSize() );
	updateEyeOffset();

	if( app::App::get()->isFrameRateEnabled() ) {
		CI_LOG_I( "Disabled framerate for better performance." );
		app::App::get()->disableFrameRate();
	}

	if( gl::isVerticalSyncEnabled() ) {
		CI_LOG_I( "Disabled vertical sync: handled by compositor service." );
		gl::enableVerticalSync( false );
	}
}

OculusRift::~OculusRift()
{
	if( mMirrorTexture ) {
		glDeleteFramebuffers( 1, &mMirrorFBO );
		ovr_DestroyMirrorTexture( mHmd, (ovrTexture*)mMirrorTexture );
	}
	if( mRenderBuffer )
		ovr_DestroySwapTextureSet( mHmd, mRenderBuffer->TextureSet );

	if( mHmd )
		ovr_Destroy( mHmd );
	
	mHmd = nullptr;
}

void OculusRift::initializeMirrorTexture( const glm::ivec2& size )
{
	OVR_VERIFY( ovr_CreateMirrorTextureGL( mHmd, GL_SRGB8_ALPHA8, size.x, size.y, (ovrTexture**)&mMirrorTexture ) );

	glGenFramebuffers( 1, &mMirrorFBO );
	glBindFramebuffer( GL_READ_FRAMEBUFFER, mMirrorFBO );
	glFramebufferTexture2D( GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mMirrorTexture->OGL.TexId, 0 );
	glFramebufferRenderbuffer( GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0 );
	glBindFramebuffer( GL_READ_FRAMEBUFFER, 0 );
}

void OculusRift::initializeFrameBuffer()
{
	auto size = mScreenPercentage * vec2( fromOvr( mHmdDesc.Resolution ) );
	mRenderBuffer = std::unique_ptr<TextureBuffer>( new TextureBuffer( mHmd, size, 1, 1 ) );
	mDepthBuffer = std::unique_ptr<DepthBuffer>( new DepthBuffer( size, 0 ) );
}

void OculusRift::calcEyePoses()
{
	ovrFrameTiming ftiming = ovr_GetFrameTiming( mHmd, 0 );
	ovrTrackingState hmdState = ovr_GetTrackingState( mHmd, ftiming.DisplayMidpointSeconds );
	ovr_CalcEyePoses( hmdState.HeadPose.ThePose, mEyeViewOffset, mEyeRenderPose );
}

void OculusRift::bind()
{
	calcEyePoses();

	if( mRenderBuffer ) {
		auto* set = mRenderBuffer->TextureSet;
		set->CurrentIndex = ( set->CurrentIndex + 1 ) % set->TextureCount;
		mRenderBuffer->setAndClearRenderSurface( mDepthBuffer.get() );
	}
}

void OculusRift::enableEye( int eyeIndex, bool applyMatrices )
{
	mProjectionCached = mViewMatrixCached = mInverseViewMatrixCached = false;
	mEye = static_cast<ovrEyeType>( eyeIndex );

	if( isPositionalTrackingEnabled() ) {
		mEyeCamera.setEyePoint( fromOvr( mEyeRenderPose[mEye].Position ) );
	}
	else {
		mEyeCamera.setEyePoint( vec3(0.0) );
	}
	mEyeCamera.setOrientation( fromOvr( mEyeRenderPose[mEye].Orientation ) );
	unsigned int projectionModifier = ovrProjection_RightHanded | ovrProjection_ClipRangeOpenGL;
	mEyeCamera.mOvrProjection = fromOvr( ovrMatrix4f_Projection( mEyeRenderDesc[mEye].Fov, mEyeCamera.getNearClip(), mEyeCamera.getFarClip(), projectionModifier ) );

	if( applyMatrices ) {
		gl::viewport( getEyeViewport() );
		gl::setModelMatrix( getModelMatrix() );
		gl::setViewMatrix( getViewMatrix() );
		gl::setProjectionMatrix( getProjectionMatrix() );
	}
}

void OculusRift::unbind()
{
	if( mRenderBuffer ) {
		mRenderBuffer->unsetRenderSurface();
	}

	submitFrame();
}

void OculusRift::submitFrame()
{
	// Set up positional data.
	ovrViewScaleDesc viewScaleDesc;
	viewScaleDesc.HmdSpaceToWorldScaleInMeters = 1.0f;

	mBaseLayer.Header.Type = ovrLayerType_EyeFov;
	mBaseLayer.Header.Flags = ovrLayerFlag_HighQuality | ovrLayerFlag_TextureOriginAtBottomLeft;
	for( auto eye : getEyes() ) {
		viewScaleDesc.HmdToEyeViewOffset[eye] = mEyeViewOffset[eye];
		mBaseLayer.Fov[eye]			= mEyeRenderDesc[eye].Fov;
		mBaseLayer.RenderPose[eye]	= mEyeRenderPose[eye];
	}
	mBaseLayer.ColorTexture[0] = mRenderBuffer->TextureSet;
	mBaseLayer.ColorTexture[1] = NULL;
	auto size = mRenderBuffer->getSize();
	mBaseLayer.Viewport[0] = { { 0, 0 }, { size.x / 2, size.y } };
	mBaseLayer.Viewport[1] = { { (size.x + 1) / 2, 0 }, { size.x / 2, size.y } };


	ovrLayerHeader* layers = &mBaseLayer.Header;
	auto result = ovr_SubmitFrame( mHmd, 0, &viewScaleDesc, &layers, 1 );
	
	mSkipFrame = ! (result == ovrSuccess);

	if( isMirrored() ) {
		// Blit mirror texture to back buffer
		glBindFramebuffer( GL_READ_FRAMEBUFFER, mMirrorFBO );
		glBindFramebuffer( GL_DRAW_FRAMEBUFFER, 0 );
		GLint w = mMirrorTexture->OGL.Header.TextureSize.w;
		GLint h = mMirrorTexture->OGL.Header.TextureSize.h;
		glBlitFramebuffer( 0, h, w, 0,
			0, 0, w, h,
			GL_COLOR_BUFFER_BIT, GL_NEAREST );
		glBindFramebuffer( GL_READ_FRAMEBUFFER, 0 );
	}
	// Do NOT advance TextureSet currentIndex - that has already been done above just before rendering.
}


std::list<ovrEyeType> OculusRift::getEyes() const
{
	if( mHmd )
		return { ovrEye_Left, ovrEye_Right };
	
	return {};
}

glm::mat4 OculusRift::getModelMatrix() const
{
	return mHostCamera.getViewMatrix();
}

glm::mat4 OculusRift::getViewMatrix() const
{
	return mEyeCamera.getViewMatrix();
}

glm::mat4 OculusRift::getProjectionMatrix() const
{
	if( ! mProjectionCached ) {
		mProjectionMatrix = mEyeCamera.getProjectionMatrix();
		mProjectionCached = true;
	}
	return mProjectionMatrix;
}

void OculusRift::enableMonoscopic( bool enabled )
{
	mIsMonoscopic = enabled;
	updateEyeOffset();
}

void OculusRift::updateEyeOffset()
{
	if( isMonoscopic() ) {
		auto centerEyeOffset = 0.5f * ( fromOvr( mEyeRenderDesc[0].HmdToEyeViewOffset ) + fromOvr( mEyeRenderDesc[1].HmdToEyeViewOffset ) );
		mEyeViewOffset[0] = toOvr( centerEyeOffset );
		mEyeViewOffset[1] = toOvr( centerEyeOffset );
	}
	else {
		mEyeViewOffset[0] = mEyeRenderDesc[0].HmdToEyeViewOffset;
		mEyeViewOffset[1] = mEyeRenderDesc[1].HmdToEyeViewOffset;
	}
}

void OculusRift::recenterPose()
{
	ovr_RecenterPose( mHmd );
}

bool OculusRift::getPositionalTrackingCamera( CameraPersp* positional ) const
{
	ovrTrackingState ts = ovr_GetTrackingState( mHmd, 0.0f );
	if( isTracked( ts ) ) {
		float aspectRatio = abs( tan( 0.5f * mHmdDesc.CameraFrustumHFovInRadians ) / tan( 0.5f * mHmdDesc.CameraFrustumVFovInRadians ) );
		positional->setPerspective(	toDegrees( mHmdDesc.CameraFrustumVFovInRadians ),
									aspectRatio,
									- mHmdDesc.CameraFrustumNearZInMeters,
									- mHmdDesc.CameraFrustumFarZInMeters );
		positional->setOrientation( fromOvr( ts.CameraPose.Orientation ) );
		positional->setEyePoint( fromOvr( ts.CameraPose.Position ) );
		return true;
	}
	return false;
}


glm::vec3 OculusRift::getLatencies() const
{
	float latencies[3] = { 0.0f, 0.0f, 0.0f };
	if( ovr_GetFloatArray( mHmd, "DK2Latency", latencies, 3 ) == 3 ) {
		return 1000.0f * glm::vec3( latencies[0], latencies[1], latencies[2] );
	}
	return glm::vec3( 0 );
}

void OculusRift::setScreenPercentage( float sp )
{
	CI_ASSERT( sp > 0.0f );
	mScreenPercentage = sp;
}

bool OculusRift::isTracked() const
{
	ovrTrackingState ts = ovr_GetTrackingState( mHmd, 0.0f );
	return isTracked( ts );
}

bool OculusRift::isTracked( const ovrTrackingState& ts ) const
{
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
	}
}

ScopedRiftBuffer::ScopedRiftBuffer( const OculusRiftRef& rift )
: mRift( rift.get() )
{
	mRift->bind();
}

ScopedRiftBuffer::~ScopedRiftBuffer()
{
	mRift->unbind();
}

 
