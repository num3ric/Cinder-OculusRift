/*
 *
 * Copyright (c) 2015
 * Paul Houx - The Barbarian Group
 * Eric Renaud-Houde - Moving Picture Company
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

#include "OVR_CAPI_0_6_0.h"
//#include "OVR_Version.h"
#include "OVR_CAPI_GL.h"

#include <map>
#include <list>
#include <mutex>

namespace hmd {

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

		GLenum internalFormat = GL_DEPTH_COMPONENT24;
		GLenum type = GL_UNSIGNED_INT;
		//if( GLE_ARB_depth_buffer_float )
		//{
		//	internalFormat = GL_DEPTH_COMPONENT32F;
		//	type = GL_FLOAT;
		//}

		glTexImage2D( GL_TEXTURE_2D, 0, internalFormat, size.x, size.y, 0, GL_DEPTH_COMPONENT, type, NULL );
	}
};

//--------------------------------------------------------------------------
struct TextureBuffer
{
	ovrSwapTextureSet* TextureSet;
	GLuint        texId;
	GLuint        fboId;
	glm::ivec2         texSize;

	TextureBuffer( ovrHmd hmd, glm::ivec2 size, int mipLevels, unsigned char * data, int sampleCount )
	{
		CI_ASSERT( sampleCount <= 1 ); // The code doesn't currently handle MSAA textures.
		texSize = size;

		// This texture isn't necessarily going to be a rendertarget, but it usually is.
		CI_ASSERT( hmd ); // No HMD? A little odd.
		CI_ASSERT( sampleCount == 1 ); // ovrHmd_CreateSwapTextureSetD3D11 doesn't support MSAA.

		auto result = ovrHmd_CreateSwapTextureSetGL( hmd, GL_RGBA, size.x, size.x, &TextureSet );
		CI_ASSERT( result == ovrSuccess );

		for( int i = 0; i < TextureSet->TextureCount; ++i ) {
			ovrGLTexture* tex = (ovrGLTexture*)&TextureSet->Textures[i];
			glBindTexture( GL_TEXTURE_2D, tex->OGL.TexId );
			glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
			glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
			glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
			glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
		}

		if( mipLevels > 1 ) {
			glGenerateMipmap( GL_TEXTURE_2D );
		}

		glGenFramebuffers( 1, &fboId );
	}

	glm::ivec2 getSize() const
	{
		return texSize;
	}

	void setAndClearRenderSurface( DepthBuffer * dbuffer )
	{
		ovrGLTexture* tex = (ovrGLTexture*)&TextureSet->Textures[TextureSet->CurrentIndex];

		glBindFramebuffer( GL_FRAMEBUFFER, fboId );
		glFramebufferTexture2D( GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex->OGL.TexId, 0 );
		glFramebufferTexture2D( GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, dbuffer->texId, 0 );

		glViewport( 0, 0, texSize.x, texSize.y );
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	}

	void unsetRenderSurface()
	{
		glBindFramebuffer( GL_FRAMEBUFFER, fboId );
		glFramebufferTexture2D( GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0 );
		glFramebufferTexture2D( GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0 );
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

	friend class OculusRift;
};

class OculusRift 
{
public:
	explicit OculusRift();
	~OculusRift();
	
	bool	initialize( const ci::app::WindowRef &window );
	void	detachFromWindow();

	//! Binds the final framebuffer used by the OVR runtime.
	void	bind() const;
	//! Unbinds the framebuffer.
	void	unbind() const;

	//! Set the base viewpoint position and orientation through the host camera.
	void	setHostCamera( const ci::CameraPersp& camera ) { mHostCamera = camera; }

	//! Returns the convenience host camera (base viewpoint position and orientation).
	ci::CameraPersp&		getHostCamera() { return mHostCamera; }
	const ci::CameraPersp&	getHostCamera() const { return mHostCamera; }

	//! Enable the current rendered eye, applies the viewport and mvp matrices directly (optionally disabled). 
	void	enableEye( int eyeIndex, bool applyMatrices = true );
	//! Returns the array of eyes indices, in the order dictated by the SDK.
	std::list<ovrEyeType>	getEyes() const;

	//! Returns the active eye camera.
	ci::CameraPersp&		getEyeCamera() { return mEyeCamera; }
	const ci::CameraPersp&	getEyeCamera() const { return mEyeCamera; }

	/*! Re-centers the sensor orientation.
	 * Normally this will recenter the (x,y,z) translational components and the yaw
	 * component of orientation. */
	void	recenterPose();

	//! Get DK2 positional tracking camera
	bool	getPositionalTrackingCamera( ci::CameraPersp* positional ) const;
	//! Get DK2 latencies (render, timewarp, post-present).
	glm::vec3	getLatencies() const;

	//! Get frame buffer screen percentage
	float	getScreenPercentage() const { return mScreenPercentage; }
	/* Set frame buffer screen percentage (fbo size / device screen size).
	 * It is best to use a value >1: downscaling a higher resolution
	 * render helps minimize aliasing and increases detail. */
	void	setScreenPercentage( float sp );

	float	getMirrorPercentage() const { return mMirrorPercentage; }

	void	setMirrorPercentage( float sp );

	//! Returns true if the render is also mirrored on screen (direct mode).
	bool	isMirrored() const;
	//! Enable mirroring to screen (direct mode).
	void	enableMirrored( bool enabled );

	//! Returns true if monoscopic (no offset between eyes).
	bool	isMonoscopic() const { return mIsMonoscopic; }
	/*! Enabling monoscopic will eliminate offsets between each eye,
	 * thus eliminating any 3d stereoscopic effect. */
	void	enableMonoscopic( bool enabled ) { mIsMonoscopic = enabled; }

	//! Returns true if the positional tracking translations are applied.
	bool	isTracked() const;
	//! Returns if positional tracking is enabled.
	bool	isPositionalTrackingEnabled() const { return mUsePositionalTracking; }
	//! Enabling the use of positional tracking translations.
	void	enablePositionalTracking( bool enabled ) { mUsePositionalTracking = enabled; }

	bool	isRenderUpdating() const { return mIsRenderUpdating; }

	//! Scale the head scale factor (based in meters)
	void	setHeadScale( float scale );

	//! Returns the native window position of the HMD.
	//glm::ivec2	getNativeWindowPos() const { return fromOvr( mHmd->WindowsPos ); }
	//! Returns the native resolution of the HMD.
	glm::ivec2	getNativeWindowResolution() const { return fromOvr( mHmd->Resolution ); }
	//! Returns the size of the render target fbo (used by both eyes).
	glm::ivec2	getFboSize() const { return mRenderBuffer->getSize(); }

	//! Returns the composed host and (active) eye view matrix.
	glm::mat4	getViewMatrix() const;
	//! Returns the composed host and (active) eye inverse view matrix.
	glm::mat4	getInverseViewMatrix() const;
	//! Returns the composed host and (active) eye projection matrix.
	glm::mat4	getProjectionMatrix() const;
	//! Returns the active eye viewport.
	std::pair<glm::ivec2, glm::ivec2> getEyeViewport() const { return fromOvr( mLayer.Viewport[mEye] ); }

	bool hasWindow( const ci::app::WindowRef &window ) const { return mWindow == window; }
//	bool isCaptured() const;
	bool isDesktopExtended() const;
private:

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
	
	static bool	isValid( const ci::app::WindowRef& window );
	
	void	initializeFrameBuffer();
	void	createMirrorTexture();

	void	updateHmdSettings();
	
	void	startDrawFn( ci::app::Renderer *renderer );
	void	finishDrawFn( ci::app::Renderer *renderer );
	

	mutable glm::mat4	mProjectionMatrix;
	mutable bool		mProjectionCached = false;
	mutable glm::mat4	mViewMatrix, mInverseViewMatrix;
	mutable bool		mViewMatrixCached = false;
	mutable bool		mInverseViewMatrixCached = false;

	ci::app::WindowRef	mWindow;

	EyeCamera			mEyeCamera;
	ci::CameraPersp		mHostCamera;

	float				mHeadScale;
	float				mScreenPercentage;
	float				mMirrorPercentage;
	unsigned int		mHmdCaps, mTrackingCaps;
	bool				mHmdSettingsChanged;
	bool				mIsExtended;
	bool				mIsMirrrored;
	bool				mIsMonoscopic;
	bool				mUsePositionalTracking;

	bool				mIsRenderUpdating;

	// Oculus Rift SDK
	ovrHmd				mHmd;
	ovrEyeType			mEye;
	ovrEyeRenderDesc	mEyeRenderDesc[ovrEye_Count];
	ovrPosef			mEyeRenderPose[ovrEye_Count];
	ovrVector3f			mEyeViewOffset[ovrEye_Count];

	ovrLayerEyeFov		mLayer;

	std::unique_ptr<TextureBuffer>	mRenderBuffer;
	std::unique_ptr<DepthBuffer>	mDepthBuffer;

	ovrGLTexture*					mMirrorTexture;
	GLuint							mMirrorFBO;
};

struct ScopedBind
{
	ScopedBind( OculusRift& rift );
	~ScopedBind();
private:
	OculusRift* mRift;
};

} // namespace hmd
 