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

#pragma once

#include "cinder/app/RendererGl.h"
#include "cinder/app/Window.h"
#include "cinder/gl/gl.h"
#include "cinder/Camera.h"
#include "cinder/Exception.h"
#include "cinder/Noncopyable.h"

#include "OVR_CAPI.h"
#include "OVR_CAPI_GL.h"

#include <map>
#include <list>
#include <mutex>

namespace hmd {

struct PoseState
{
	PoseState( const ovrPoseStatef& ps );

	glm::vec3   position;
	glm::quat	orientation;
	glm::vec3	angularVelocity;
	glm::vec3	linearVelocity;
	glm::vec3	angularAcceleration;
	glm::vec3	linearAcceleration;
	double		timeInSeconds;
};

///////////////////// CONVERSIONS ///////////////////////////

inline glm::ivec2 fromOvr( const ovrSizei& v )
{
	// faster than glm::make_*
	return glm::ivec2( v.w, v.h );
}

inline glm::ivec2 fromOvr( const ovrVector2i& v )
{
	// faster than glm::make_*
	return glm::ivec2( v.x, v.y );
}

inline glm::vec2 fromOvr( const ovrVector2f& v )
{
	// faster than glm::make_*
	return glm::vec2( v.x, v.y );
}

inline glm::vec3 fromOvr( const ovrVector3f& v )
{
	// faster than glm::make_*
	return glm::vec3( v.x, v.y, v.z );
}

inline glm::mat4 fromOvr( const ovrMatrix4f& m )
{
	return glm::transpose( glm::make_mat4( &m.M[0][0] ) );
}

inline glm::quat fromOvr( const ovrQuatf& q )
{
	return glm::quat( q.w, q.x, q.y, q.z );
}

inline glm::mat4 fromOvr( const ovrPosef& p )
{
	return glm::toMat4( fromOvr( p.Orientation ) ) * glm::translate( fromOvr( p.Position ) );
}

inline std::pair<glm::ivec2, glm::ivec2> fromOvr( const ovrRecti& r )
{
	return std::pair<glm::ivec2, glm::ivec2>( fromOvr( r.Pos ), fromOvr( r.Size ) );
}

inline ovrMatrix4f toOvr( const glm::mat4& m )
{
	ovrMatrix4f result;
	glm::mat4 mat = glm::transpose( m );
	memcpy( result.M, &mat, sizeof( glm::mat4 ) );
	return result;
}

inline ovrVector3f toOvr( const glm::vec3& v )
{
	ovrVector3f result;
	result.x = v.x;
	result.y = v.y;
	result.z = v.z;
	return result;
}

inline ovrVector2f toOvr( const glm::vec2& v )
{
	ovrVector2f result;
	result.x = v.x;
	result.y = v.y;
	return result;
}

inline ovrSizei toOvr( const glm::ivec2& v )
{
	ovrSizei result;
	result.w = v.x;
	result.h = v.y;
	return result;
}

inline ovrQuatf toOvr( const glm::quat& q )
{
	ovrQuatf result;
	result.x = q.x;
	result.y = q.y;
	result.z = q.z;
	result.w = q.w;
	return result;
}

static const float kLeapToRiftScale = 0.01f;
static const glm::vec3 kLeapToRiftEuler = glm::vec3( -0.5f * (float)M_PI, 0, (float)M_PI );

////////////////////////////////////////////////////////////////////////


// TODO: Cinder gl::Fbos currently don't allow for changing the texture attachments at every frame.
// We thus use the provided sample implementation by Oculus.

//---------------------------------------------------------------------------------------
struct DepthBuffer
{
	GLuint        texId;

	DepthBuffer( glm::ivec2 size, int sampleCount )
	{
		CI_ASSERT( sampleCount <= 1 ); // The code doesn't currently handle MSAA textures.

		glGenTextures( 1, &texId );
		glBindTexture( GL_TEXTURE_2D, texId );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
		glTexImage2D( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, size.x, size.y, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, NULL );
	}

	~DepthBuffer() {
		glDeleteTextures( 1, &texId );
	}
};

//--------------------------------------------------------------------------
struct TextureBuffer
{
	ovrSession          mSession;
	ovrTextureSwapChain	mTextureChain;
	GLuint				mTexId;
	GLuint				mFboId;
	glm::ivec2			mSize;

	TextureBuffer( ovrSession session, glm::ivec2 size, int mipLevels, int sampleCount )
		: mSession( session )
	{
		CI_ASSERT( sampleCount <= 1 ); // The code doesn't currently handle MSAA textures.
		mSize = size;

		// This texture isn't necessarily going to be a rendertarget, but it usually is.
		CI_ASSERT( session );
		CI_ASSERT( sampleCount == 1 ); // ovrHmd_CreateSwapTextureSetD3D11 doesn't support MSAA.

        ovrTextureSwapChainDesc desc = {};
        desc.Type = ovrTexture_2D;
        desc.ArraySize = 1;
        desc.Width = mSize.x;
        desc.Height = mSize.y;
        desc.MipLevels = 1;
        desc.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
        desc.SampleCount = 1;
        desc.StaticImage = ovrFalse;

        ovrResult result = ovr_CreateTextureSwapChainGL( mSession, &desc, &mTextureChain );

        int length = 0;
		ovr_GetTextureSwapChainLength( mSession, mTextureChain, &length );
		CI_ASSERT( result == ovrSuccess );

		for( int i = 0; i < length; ++i ) {
			GLuint chainTexId;
			ovr_GetTextureSwapChainBufferGL( mSession, mTextureChain, i, &chainTexId );
			glBindTexture( GL_TEXTURE_2D, chainTexId );
			glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
			glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
			glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
			glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
		}

		if( mipLevels > 1 ) {
			glGenerateMipmap( GL_TEXTURE_2D );
		}

		glGenFramebuffers( 1, &mFboId );
	}

	~TextureBuffer() {
		if( mTextureChain ) {
			ovr_DestroyTextureSwapChain( mSession, mTextureChain );
			mTextureChain = nullptr;
		}
		if( mTexId ) {
			glDeleteTextures( 1, &mTexId );
			mTexId = 0;
		}
		if( mFboId ) {
			glDeleteFramebuffers( 1, &mFboId );
			mFboId = 0;
		}
	}

	glm::ivec2 getSize() const
	{
		return mSize;
	}

	void setAndClearRenderSurface( DepthBuffer * dbuffer )
	{
		GLuint curTexId;
		if( mTextureChain ) {
			int curIndex;
			ovr_GetTextureSwapChainCurrentIndex( mSession, mTextureChain, &curIndex );
			ovr_GetTextureSwapChainBufferGL( mSession, mTextureChain, curIndex, &curTexId );
		} else {
			curTexId = mTexId;
		}

		glBindFramebuffer( GL_FRAMEBUFFER, mFboId );
		glFramebufferTexture2D( GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, curTexId, 0 );
		glFramebufferTexture2D( GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, dbuffer->texId, 0 );

		glViewport( 0, 0, mSize.x, mSize.y );
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

		//glEnable( GL_FRAMEBUFFER_SRGB );
	}

	void unsetRenderSurface()
	{
		glBindFramebuffer( GL_FRAMEBUFFER, mFboId );
		glFramebufferTexture2D( GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0 );
		glFramebufferTexture2D( GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0 );
	}

	void commit() {
		if( mTextureChain ) {
			ovr_CommitTextureSwapChain( mSession, mTextureChain );
		}
	}
};


class OculusRift;

class RiftManager : public ci::Noncopyable 
{
public:
	~RiftManager();	
	static void initialize();
private:
	RiftManager();
	static std::once_flag				mOnceFlag;
	static std::unique_ptr<RiftManager>	mInstance;

	friend OculusRift;
};

typedef std::shared_ptr<OculusRift> OculusRiftRef;

class OculusRift : ci::Noncopyable
{
public:
	struct Params {
		Params();
		Params& screenPercentage( float sp ) { mScreenPercentage = sp; return *this; }
		Params& hostCamera( const ci::CameraPersp& host ) { mHostCam = { true, host }; return *this; }
		Params& monoscopic( bool mono ) { mIsMonoscopic = mono; return *this; }
		Params& mirrored( bool mirror ) { mIsMirrrored = mirror; return *this; }
		Params& clipDistance(float nearClip, float farClip) { mClipDistance = { true, glm::vec2(nearClip, farClip) };  return *this; }
		Params& positional( bool tracked ) { mUsePositionalTracking = tracked; return *this; }
	private:
		std::pair<bool, ci::CameraPersp> mHostCam;
		std::pair<bool, glm::vec2> mClipDistance;
		float mScreenPercentage = 1.3f;
		bool mIsMonoscopic = false;
		bool mIsMirrrored = true;
		bool mUsePositionalTracking = true;
		friend OculusRift;
	};


	static OculusRiftRef create( const Params& params = Params{} );
	~OculusRift();
	//! Binds the final framebuffer used by the OVR runtime.
	void	bind();
	//! Unbinds the framebuffer.
	void	unbind();

	//! Returns the convenience host camera (base viewpoint position and orientation).
	const ci::CameraPersp&	getHostCamera() const { return mHostCamera; }
	//! Set the base viewpoint position and orientation through the host camera. (Affects the model matrix.)
	void	setHostCamera( const ci::CameraPersp& camera ) { mModelMatrixCached = false; mHostCamera = camera; }

	//! Enable the current rendered eye, applies the viewport and mvp matrices directly (optionally disabled). 
	void	enableEye( int eyeIndex, bool applyMatrices = true );
	//! Returns the array of eyes indices, in the order dictated by the SDK.
	std::list<ovrEyeType>	getEyes() const;

	//! Returns the active eye camera.
	const ci::CameraPersp&	getEyeCamera() const { return mEyeCamera; }

	/*! Re-centers the sensor orientation.
	 * Normally this will recenter the (x,y,z) translational components and the yaw
	 * component of orientation. */
	void	recenterPose();

	//! Get DK2 positional tracking camera
	bool	getPositionalTrackingCamera( ci::CameraPersp* positional ) const;

	//! Get hand pose information if tracked.
	bool getHandPose( int handIndex, PoseState* ps ) const;

	//! Get DK2 latencies (render, timewarp, post-present).
	glm::vec3	getLatencies() const;

	//! Get frame buffer screen percentage
	float	getScreenPercentage() const { return mScreenPercentage; }
	/* Set frame buffer screen percentage (fbo size / device screen size).
	 * It is best to use a value >1: downscaling a higher resolution
	 * render helps minimize aliasing and increases detail. */
	void	setScreenPercentage( float sp );

	//! Returns true if the render is also mirrored on screen.
	bool	isMirrored() const;
	//! Enable mirroring to screen.
	void	enableMirrored( bool enabled );

	//! Returns true if monoscopic (no offset between eyes).
	bool	isMonoscopic() const { return mIsMonoscopic; }
	/*! Enabling monoscopic will eliminate offsets between each eye,
	 * thus eliminating any 3d stereoscopic effect. */
	void	enableMonoscopic( bool enabled );

	//! Updates horizontal offset between the eyes depending on mono vs stereo rendering.
	void	updateEyeOffset();

	//! Returns true if the positional tracking translations are applied.
	bool	isTracked() const;
	bool	isTracked( const ovrTrackingState& ts ) const;
	//! Returns if positional tracking is enabled.
	bool	isPositionalTrackingEnabled() const { return mUsePositionalTracking; }
	//! Enabling the use of positional tracking translations.
	void	enablePositionalTracking( bool enabled ) { mUsePositionalTracking = enabled; }
	//! Cycles through the performance compositor huds.
	void	cyclePerfHudModes( bool enabled = true );

	//! Returns the native resolution of the HMD.
	glm::ivec2	getNativeHmdResolution() const { return fromOvr( mHmdDesc.Resolution ); }
	//! Returns the size of the render target fbo (used by both eyes).
	glm::ivec2	getFboSize() const { return mRenderBuffer->getSize(); }

	//! Returns the host camera view matrix, which acts as the rift model matrix.
	inline glm::mat4	getModelMatrix() const;
	//! Returns the active eye's view matrix.
	inline glm::mat4	getViewMatrix() const;
	//! Returns the composed host and (active) eye projection matrix.
	glm::mat4	getProjectionMatrix() const;
	//! Returns the eye viewport.
	ci::Area getEyeViewport( int eyeIndex ) const;

private:
	explicit OculusRift( const Params& params );

	class EyeCamera : public ci::CameraPersp 
	{
	protected:
		void calcProjection() const override
		{
			mProjectionMatrix = mOvrProjection;
			mProjectionCached = true;
		}
	private:
		glm::mat4		mOvrProjection;

		friend class OculusRift;
	};
	
	void	initializeFrameBuffer();
	void	initializeMirrorTexture( const glm::ivec2& size );
	void	destroyMirrorTexture();
	void	submitFrame();
	
	mutable glm::mat4	mModelMatrix;
	mutable bool		mModelMatrixCached = false;
	mutable glm::mat4	mProjectionMatrix;
	mutable bool		mProjectionCached = false;
	mutable glm::mat4	mViewMatrix, mInverseViewMatrix;
	mutable bool		mViewMatrixCached = false;
	mutable bool		mInverseViewMatrixCached = false;

	EyeCamera			mEyeCamera;
	ci::CameraPersp		mHostCamera;

	float				mScreenPercentage;
	unsigned int		mTrackingCaps;
	bool				mIsMirrrored;
	bool				mIsMonoscopic;
	bool				mUsePositionalTracking;

	bool				mIsVisible;

	// Oculus Rift SDK
	long long			mFrameIndex;
	ovrSession			mSession;
	ovrHmdDesc			mHmdDesc;
	ovrEyeType			mEye;
	ovrEyeRenderDesc	mEyeRenderDesc[ovrEye_Count];
	ovrPosef			mEyeRenderPose[ovrEye_Count];
	ovrVector3f			mEyeViewOffset[ovrEye_Count];
	ovrLayerEyeFov		mBaseLayer;

	std::unique_ptr<TextureBuffer>	mRenderBuffer;
	std::unique_ptr<DepthBuffer>	mDepthBuffer;

	ovrMirrorTexture				mMirrorTexture;
	GLuint							mMirrorFBO;

	double							mSensorSampleTime;

	int								mPerfHudMode;
};

struct ScopedRiftBuffer
{
	ScopedRiftBuffer( const OculusRiftRef& rift );
	~ScopedRiftBuffer();
private:
	OculusRift* mRift;
};

class RiftExeption : public ci::Exception {
public:
	RiftExeption() { }
	RiftExeption( const std::string &description ) : Exception( description ) { }
};

} // namespace hmd
 