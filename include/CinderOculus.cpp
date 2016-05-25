/*
 *
 * Copyright (c) 2015
 * Paul Houx - The Barbarian Group
 * Eric Renaud-Houde - Moving Picture Company / The Mill
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

PoseState::PoseState( const ovrPoseStatef& ps )
: position{ fromOvr( ps.ThePose.Position ) }
, orientation{ fromOvr( ps.ThePose.Orientation ) }
, angularVelocity{ fromOvr( ps.AngularVelocity ) }
, angularAcceleration{ fromOvr( ps.AngularAcceleration ) }
, linearVelocity{ fromOvr( ps.LinearVelocity ) }
, linearAcceleration{ fromOvr( ps.LinearAcceleration ) }
, timeInSeconds{ ps.TimeInSeconds }
{

}

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

OculusRift::Params::Params()
: mHostCam{ false, ci::CameraPersp{} }
, mClipDistance{ false, vec2(-1) }
{

}

OculusRiftRef OculusRift::create( const Params& params )
{
	return OculusRiftRef( new OculusRift{ params } );
}

OculusRift::OculusRift( const Params& params )
: mScreenPercentage( params.mScreenPercentage )
, mIsMirrrored( params.mIsMirrrored )
, mIsMonoscopic( params.mIsMonoscopic )
, mUsePositionalTracking( params.mUsePositionalTracking )
, mFrameIndex( 0 )
, mSession( nullptr )
, mMirrorFBO( 0 )
, mMirrorTexture( nullptr )
, mIsVisible( true )
, mPerfHudMode( 0 )
, mSensorSampleTime( 0 )
{
	if( params.mHostCam.first ) {
		mHostCamera = params.mHostCam.second;
	}
	else {
		mHostCamera.setEyePoint( vec3( 0 ) );
		mHostCamera.setViewDirection( vec3( 0, 0, -1 ) );
	}

	if( params.mClipDistance.first ) {
		mEyeCamera.setNearClip( params.mClipDistance.second.x );
		mEyeCamera.setFarClip( params.mClipDistance.second.y );
	}
	
	ovrGraphicsLuid luid; //can't use in opengl
	OVR_VERIFY( ovr_Create( &mSession, &luid ) );
	mHmdDesc = ovr_GetHmdDesc( mSession );

	for( int i = 0; i < ovrEye_Count; ++i ) {
		mEyeRenderDesc[i] = ovr_GetRenderDesc( mSession, (ovrEyeType)i, mHmdDesc.DefaultEyeFov[i] );
	}

	initializeFrameBuffer();
	updateEyeOffset();
	app::getWindow()->getSignalResize().connect( [this](){
		initializeMirrorTexture( app::getWindowSize() );
	} );

	//ovr_SetTrackingOriginType( mSession, ovrTrackingOrigin_EyeLevel );

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
	//To prevent SDK bug(?) where perf hud state is maintained by the runtime
	cyclePerfHudModes( false );

	if( mMirrorTexture )
		destroyMirrorTexture();

	mRenderBuffer.reset();
	mDepthBuffer.reset();

	if( mSession )
		ovr_Destroy( mSession );
	
	mSession = nullptr;
}

void OculusRift::initializeMirrorTexture( const glm::ivec2& size )
{
	if( mMirrorTexture )
		destroyMirrorTexture();

	ovrMirrorTextureDesc desc;
	memset( &desc, 0, sizeof( desc ) );
	desc.Width = size.x;
	desc.Height = size.y;
	desc.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;

	// Create mirror texture and an FBO used to copy mirror texture to back buffer
	ovrResult result = ovr_CreateMirrorTextureGL( mSession, &desc, &mMirrorTexture );
	if( ! OVR_SUCCESS( result ) ) {
		CI_LOG_E( "Failed to create mirror texture." );
	}

	// Configure the mirror read buffer
	GLuint texId;
	ovr_GetMirrorTextureBufferGL( mSession, mMirrorTexture, &texId );

	glGenFramebuffers( 1, &mMirrorFBO );
	glBindFramebuffer( GL_READ_FRAMEBUFFER, mMirrorFBO );
	glFramebufferTexture2D( GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texId, 0 );
	glFramebufferRenderbuffer( GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0 );
	glBindFramebuffer( GL_READ_FRAMEBUFFER, 0 );
}

void OculusRift::destroyMirrorTexture()
{
	if( mMirrorFBO )
		glDeleteFramebuffers( 1, &mMirrorFBO );
	if( mMirrorTexture )
		ovr_DestroyMirrorTexture( mSession, mMirrorTexture );
}

void OculusRift::initializeFrameBuffer()
{
	auto size = mScreenPercentage * vec2( fromOvr( mHmdDesc.Resolution ) );
	mRenderBuffer = std::unique_ptr<TextureBuffer>( new TextureBuffer( mSession, size, 1, 1 ) );
	mDepthBuffer = std::unique_ptr<DepthBuffer>( new DepthBuffer( size, 0 ) );

	if( ! mRenderBuffer->mTextureChain ) {
		CI_LOG_E( "Failed to create texture." );
	}
}

void OculusRift::bind()
{
	ovr_GetEyePoses( mSession, mFrameIndex, ovrTrue, mEyeViewOffset, mEyeRenderPose, &mSensorSampleTime );

	if( mRenderBuffer && mIsVisible ) {
		mRenderBuffer->setAndClearRenderSurface( mDepthBuffer.get() );
	}
}

void OculusRift::enableEye( int eyeIndex, bool applyMatrices )
{
	mEye = static_cast<ovrEyeType>( eyeIndex );

	if( isPositionalTrackingEnabled() ) {
		mEyeCamera.setEyePoint( fromOvr( mEyeRenderPose[mEye].Position ) );
	}
	else {
		mEyeCamera.setEyePoint( vec3(0.0) );
	}
	mEyeCamera.setOrientation( fromOvr( mEyeRenderPose[mEye].Orientation ) );
	mEyeCamera.mOvrProjection = fromOvr( ovrMatrix4f_Projection( mEyeRenderDesc[mEye].Fov, mEyeCamera.getNearClip(), mEyeCamera.getFarClip(), ovrProjection_None ) );

	if( applyMatrices ) {
		auto area = getEyeViewport( eyeIndex );
		gl::viewport( area.getUL(), area.getSize() );
		gl::setModelMatrix( getModelMatrix() );
		gl::setViewMatrix( getViewMatrix() );
		gl::setProjectionMatrix( mEyeCamera.mOvrProjection );
	}
}

void OculusRift::unbind()
{
	if( mRenderBuffer && mIsVisible ) {
		mRenderBuffer->unsetRenderSurface();
		mRenderBuffer->commit();
	}

	submitFrame();
}

void OculusRift::submitFrame()
{
	// Set up positional data.
	ovrViewScaleDesc viewScaleDesc;
	viewScaleDesc.HmdSpaceToWorldScaleInMeters = 1.0f;

	mBaseLayer.Header.Type = ovrLayerType_EyeFov;
	mBaseLayer.Header.Flags = ovrLayerFlag_TextureOriginAtBottomLeft;
	mBaseLayer.SensorSampleTime = mSensorSampleTime;
	for( auto eye : getEyes() ) {
		viewScaleDesc.HmdToEyeOffset[eye] = mEyeViewOffset[eye];
		mBaseLayer.Fov[eye]			= mEyeRenderDesc[eye].Fov;
		mBaseLayer.RenderPose[eye]	= mEyeRenderPose[eye];
		auto area = getEyeViewport( eye );
		auto sz = area.getSize();
		mBaseLayer.Viewport[eye] = { { area.x1, area.y1 }, { sz.x, sz.y } };
	}
	mBaseLayer.ColorTexture[0] = mRenderBuffer->mTextureChain;
	mBaseLayer.ColorTexture[1] = NULL;
	

	ovrLayerHeader* layers = &mBaseLayer.Header;
	auto result = ovr_SubmitFrame( mSession, mFrameIndex, &viewScaleDesc, &layers, 1 );
	mIsVisible = (result == ovrSuccess);
	
	if( mMirrorTexture && isMirrored() ) {
		// Blit mirror texture to back buffer
		glBindFramebuffer( GL_READ_FRAMEBUFFER, mMirrorFBO );
		glBindFramebuffer( GL_DRAW_FRAMEBUFFER, 0 );
		GLint w = app::getWindowWidth();
		GLint h = app::getWindowHeight();
		glBlitFramebuffer( 0, h, w, 0,
			0, 0, w, h,
			GL_COLOR_BUFFER_BIT, GL_NEAREST );
		glBindFramebuffer( GL_READ_FRAMEBUFFER, 0 );
	}

	++mFrameIndex;
}


std::list<ovrEyeType> OculusRift::getEyes() const
{
	if( mSession && mIsVisible )
		return { ovrEye_Left, ovrEye_Right };
	
	return {};
}

Area OculusRift::getEyeViewport( int eyeIndex ) const
{
	auto size = mRenderBuffer->getSize();
	if( eyeIndex == 0 ) {
		return Area{ 0, 0, size.x / 2, size.y };
	}
	return Area{ ( size.x + 1) / 2, 0, size.x, size.y };
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
	return mEyeCamera.mOvrProjection;
}

void OculusRift::enableMonoscopic( bool enabled )
{
	mIsMonoscopic = enabled;
	updateEyeOffset();
}

void OculusRift::updateEyeOffset()
{
	if( isMonoscopic() ) {
		auto centerEyeOffset = 0.5f * ( fromOvr( mEyeRenderDesc[0].HmdToEyeOffset ) + fromOvr( mEyeRenderDesc[1].HmdToEyeOffset ) );
		mEyeViewOffset[0] = toOvr( centerEyeOffset );
		mEyeViewOffset[1] = toOvr( centerEyeOffset );
	}
	else {
		mEyeViewOffset[0] = mEyeRenderDesc[0].HmdToEyeOffset;
		mEyeViewOffset[1] = mEyeRenderDesc[1].HmdToEyeOffset;
	}
}

void OculusRift::recenterPose()
{
	ovr_RecenterTrackingOrigin( mSession );
}

bool OculusRift::getPositionalTrackingCamera( CameraPersp* positional ) const
{
	ovrTrackingState ts = ovr_GetTrackingState( mSession, 0.0f, ovrFalse );
	if( isTracked( ts ) ) {
		
		ovrTrackerDesc td = ovr_GetTrackerDesc( mSession, 0 );
		ovrTrackerPose tp = ovr_GetTrackerPose( mSession, 0 );

		float aspectRatio = abs( tan( 0.5f * td.FrustumHFovInRadians ) / tan( 0.5f * td.FrustumVFovInRadians ) );
		positional->setPerspective(	toDegrees( td.FrustumVFovInRadians ),
									aspectRatio,
									- td.FrustumFarZInMeters,
									- td.FrustumFarZInMeters );
		positional->setOrientation( fromOvr( tp.Pose.Orientation ) );
		positional->setEyePoint( fromOvr( tp.Pose.Position ) );
		return true;
	}
	return false;
}

bool OculusRift::getHandPose( int handIndex, PoseState* ps ) const
{
	ovrTrackingState ts = ovr_GetTrackingState( mSession, 0.0f, ovrFalse );
	ovrHandType hi = static_cast<ovrHandType>( handIndex );
	if( (ts.HandStatusFlags[hi] & ovrStatus_OrientationTracked) && (ts.HandStatusFlags[hi] & ovrStatus_PositionTracked) ) {
		*ps = PoseState{ ts.HandPoses[hi] };
		return true;
	}
	return false;
}

glm::vec3 OculusRift::getLatencies() const
{
	float latencies[3] = { 0.0f, 0.0f, 0.0f };
	if( ovr_GetFloatArray( mSession, "DK2Latency", latencies, 3 ) == 3 ) {
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
	ovrTrackingState ts = ovr_GetTrackingState( mSession, 0.0f, ovrFalse );
	return isTracked( ts );
}

bool OculusRift::isTracked( const ovrTrackingState& ts ) const
{
	bool tracked = ts.StatusFlags & ovrStatus_PositionTracked;
	return tracked && isPositionalTrackingEnabled();
}

void OculusRift::cyclePerfHudModes( bool enabled )
{
	mPerfHudMode = ( enabled ) ? (++mPerfHudMode) % int(ovrPerfHud_Count) : 0;
	ovr_SetInt( mSession, OVR_PERF_HUD_MODE, mPerfHudMode );
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

 
