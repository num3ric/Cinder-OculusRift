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
	// Disabled after update to SDK 0.5.0.1 to prevent including LibOVRKernel with this block.
	// Are these thread settings still desirable?

//	OVR::Thread::SetCurrentThreadName( "CiOculusMain" );
//	OVR::Thread::SetCurrentPriority( OVR::Thread::HighestPriority );
//#if defined( OVR_OS_WIN32 )
//	if( OVR::Thread::GetCPUCount() >= 4 ) {
//		SetPriorityClass( GetCurrentProcess(), HIGH_PRIORITY_CLASS );
//	}
//#endif

	ovrBool result = ovr_Initialize();
	if( ! result ) {
		throw std::runtime_error( "Failed to initialize Oculus VR." );
	}
}

RiftManager::~RiftManager()
{
	ovr_Shutdown();
}

class OculusRift::ModeImpl {
public:
	ModeImpl( OculusRift * rift )
		: mRift( rift ) { }
	virtual ~ModeImpl() { }
	virtual bool initialize( const WindowRef& window ) = 0;
	virtual ovrFrameTiming beginFrame() = 0;
	virtual void endFrame( Renderer *renderer ) = 0;
protected:
	OculusRift * mRift;
};

class OculusRift::DirectModeImpl : public OculusRift::ModeImpl {
public:
	DirectModeImpl( OculusRift * rift )
		: ModeImpl( rift )
	{ }
	bool initialize( const WindowRef& window ) override
	{
		// Attach to window.
		if( ! ovrHmd_AttachToWindow( mRift->mHmd, window->getNative(), nullptr, nullptr ) ) {
			return false;
		}
		// Create or resize the frame buffer.
		mRift->initializeFrameBuffer();
		return true;
	}
	ovrFrameTiming beginFrame() override
	{
		return ovrHmd_BeginFrame( mRift->mHmd, 0 );
	}
	void endFrame( Renderer *renderer ) override
	{
		ovrHmd_EndFrame( mRift->mHmd, mRift->mEyeRenderPose, &mRift->mEyeTexture[0].Texture );
	}
private:
	
};

class OculusRift::ExtendedModeImpl : public OculusRift::ModeImpl {
public:
	ExtendedModeImpl( OculusRift * rift )
		: ModeImpl( rift )
	{ }
	bool initialize( const WindowRef& window ) override
	{
		// Make sure our window is positioned and sized correctly.
		window->setBorderless( true );
		window->setPos( mRift->getNativeWindowPos() );
		window->setSize( mRift->getNativeWindowResolution() );
		// Create or resize the frame buffer.
		mRift->initializeFrameBuffer();
		initializeDistortion();
		return true;
	}

	ovrFrameTiming beginFrame() override
	{
		return ovrHmd_BeginFrameTiming( mRift->mHmd, 0 );
	}
	
	void endFrame( Renderer *renderer ) override
	{
		gl::viewport( mRift->getNativeWindowResolution() );
		gl::setMatricesWindow( mRift->getNativeWindowResolution() );
		gl::clear();

//		gl::color( 1, 1, 1 );
//		gl::disableAlphaBlending();

		// Bind frame buffer texture.
		gl::ScopedTextureBind tex0( mRift->mFbo->getColorTexture(), (uint8_t)0 );

		// Render distortion mesh for each eye.
		for( ovrEyeType eye = ovrEye_Left; eye < ovrEye_Count; eye = static_cast<ovrEyeType>( eye + 1 ) ) {
			gl::ScopedGlslProg glsl( mShader );
			mShader->uniform( "Texture", 0 );
			mShader->uniform( "EyeToSourceUVScale", fromOvr( mRift->mUVScaleOffset[eye][0] ) * vec2( 1, -1 ) ); // Flip texture.
			mShader->uniform( "EyeToSourceUVOffset", fromOvr( mRift->mUVScaleOffset[eye][1] ) );

			ovrMatrix4f timeWarpMatrices[2];
			ovrHmd_GetEyeTimewarpMatrices( mRift->mHmd, eye, mRift->mEyeRenderPose[eye], timeWarpMatrices );
			mShader->uniform( "EyeRotationStart", fromOvr( timeWarpMatrices[0] ) );
			mShader->uniform( "EyeRotationEnd", fromOvr( timeWarpMatrices[1] ) );

			mBatch[eye]->draw();
		}
		// Draw latency indicator.
		unsigned char latencyColor[3];
		ovrBool drawDk2LatencyQuad = ovrHmd_GetLatencyTest2DrawColor( mRift->mHmd, latencyColor );
		if( drawDk2LatencyQuad ) {
			const float latencyQuadSize = 20; // only needs to be 1-pixel, but larger helps visual debugging
			gl::ScopedColor color( Color( latencyColor[0] / 255.0f, latencyColor[1] / 255.0f, latencyColor[2] / 255.0f ) );
			gl::drawSolidRect( Rectf( float( mRift->getNativeWindowResolution().x - latencyQuadSize ), 0, float( mRift->getNativeWindowResolution().x ), latencyQuadSize ) );
		}

		renderer->swapBuffers();

		ovrHmd_EndFrameTiming( mRift->mHmd );
	}
private:
	// Distortion vertex shader.
	const std::string riftVertexShader =
		"#version 150\n"
		""
		"uniform vec2 EyeToSourceUVScale, EyeToSourceUVOffset;\n"
		"uniform mat4 EyeRotationStart, EyeRotationEnd;\n"
		""
		"in vec4 ciPosition;\n"
		"in vec2 ciTexCoord0;\n"
		"in vec2 ciTexCoord1;\n"
		"in vec2 ciTexCoord2;\n"
		""
		"out vec2 oTexCoord0;\n"
		"out vec2 oTexCoord1;\n"
		"out vec2 oTexCoord2;\n"
		"out float oVignette;\n"
		""
		"vec2 TimewarpTexCoord( in vec2 TexCoord, in float timewarpLerpFactor )\n"
		"{\n"
		// Vertex inputs are in TanEyeAngle space for the R,G,B channels (i.e. after chromatic 
		// aberration and distortion). These are now "real world" vectors in direction (x,y,1) 
		// relative to the eye of the HMD.	Apply the 3x3 timewarp rotation to these vectors.
		"    vec4 coord = vec4( TexCoord, 1, 1 );\n"
		"    vec3 coordStart = ( EyeRotationStart * coord ).xyz;\n"
		"    vec3 coordEnd = ( EyeRotationEnd * coord ).xyz;\n"
		"    vec3 transformed = mix( coordStart, coordEnd, timewarpLerpFactor );\n"
		// Project them back onto the Z=1 plane of the rendered images.
		"    vec2 flattened = (transformed.xy / transformed.z);\n"
		// Scale them into ([0,0.5],[0,1]) or ([0.5,0],[0,1]) UV lookup space (depending on eye)
		"    return(EyeToSourceUVScale * flattened + EyeToSourceUVOffset);\n"
		"}\n"

		"void main(void)\n"
		"{\n"
		"    oTexCoord0  = TimewarpTexCoord( ciTexCoord0, ciPosition.z );\n"
		"    oTexCoord1  = TimewarpTexCoord( ciTexCoord1, ciPosition.z );\n"
		"    oTexCoord2  = TimewarpTexCoord( ciTexCoord2, ciPosition.z );\n"
		"    oVignette   = ciPosition.w;\n"
		"    gl_Position = vec4( ciPosition.xy, 0.5, 1.0 );\n"
		"}";

	// Distortion fragment shader.
	const std::string riftFragmentShader =
		"#version 150\n"
		""
		"uniform sampler2D Texture;\n"
		""
		"in vec2 oTexCoord0;\n"
		"in vec2 oTexCoord1;\n"
		"in vec2 oTexCoord2;\n"
		"in float oVignette;\n"
		""
		"out vec4 oColor;\n"
		""
		"void main(void)\n"
		"{\n"
		// 3 samples for fixing chromatic aberrations
		"    float R = texture( Texture, oTexCoord0.xy ).r;\n"
		"    float G = texture( Texture, oTexCoord1.xy ).g;\n"
		"    float B = texture( Texture, oTexCoord2.xy ).b;\n"
		"    oColor = oVignette * vec4( R, G, B, 1);\n"
		"}";

	void initializeDistortion()
	{
		try {
			mShader = gl::GlslProg::create( riftVertexShader, riftFragmentShader );
		}
		catch( const gl::GlslProgCompileExc& e ) {
			CI_LOG_E( e.what() );
		}

		gl::VboMesh::Layout layout;
		layout.attrib( geom::POSITION, 4 );
		//layout.attrib( geom::CUSTOM_0, 1 ); // Encoded in position.z instead.
		//layout.attrib( geom::CUSTOM_1, 1 ); // Encoded in position.w instead.
		layout.attrib( geom::TEX_COORD_0, 2 );
		layout.attrib( geom::TEX_COORD_1, 2 );
		layout.attrib( geom::TEX_COORD_2, 2 );
		layout.interleave( true );
		layout.usage( GL_STATIC_DRAW );

		geom::BufferLayout bufferLayout;
		bufferLayout.append( geom::POSITION, 4, sizeof( ovrDistortionVertex ), offsetof( ovrDistortionVertex, ScreenPosNDC ), 0 );
		//bufferLayout.append( geom::CUSTOM_0, 1, sizeof( ovrDistortionVertex ), offsetof( ovrDistortionVertex, TimeWarpFactor ), 0 ); // Encoded in position.z instead.
		//bufferLayout.append( geom::CUSTOM_1, 1, sizeof( ovrDistortionVertex ), offsetof( ovrDistortionVertex, VignetteFactor ), 0 ); // Encoded in position.w instead.
		bufferLayout.append( geom::TEX_COORD_0, 2, sizeof( ovrDistortionVertex ), offsetof( ovrDistortionVertex, TanEyeAnglesR ), 0 );
		bufferLayout.append( geom::TEX_COORD_1, 2, sizeof( ovrDistortionVertex ), offsetof( ovrDistortionVertex, TanEyeAnglesG ), 0 );
		bufferLayout.append( geom::TEX_COORD_2, 2, sizeof( ovrDistortionVertex ), offsetof( ovrDistortionVertex, TanEyeAnglesB ), 0 );

		ovrFovPort eyeFov[2] = { mRift->mHmd->DefaultEyeFov[0], mRift->mHmd->DefaultEyeFov[1] };
		for( ovrEyeType eye = ovrEye_Left; eye < ovrEye_Count; eye = static_cast<ovrEyeType>( eye + 1 ) ) {
			ovrDistortionMesh meshData;
			ovrHmd_CreateDistortionMesh( mRift->mHmd, eye, eyeFov[eye], ovrDistortionCap_TimeWarp, &meshData );

			gl::VboMeshRef mesh = gl::VboMesh::create( meshData.VertexCount, GL_TRIANGLES, { layout }, meshData.IndexCount, GL_UNSIGNED_SHORT );
			gl::VboRef buffer = gl::Vbo::create( GL_ARRAY_BUFFER, sizeof(ovrDistortionVertex)* meshData.VertexCount, meshData.pVertexData, GL_STATIC_DRAW );
			mesh->appendVbo( bufferLayout, buffer );
			mesh->bufferIndices( sizeof(unsigned short)* meshData.IndexCount, meshData.pIndexData );

			ovrHmd_DestroyDistortionMesh( &meshData );

			mBatch[eye] = gl::Batch::create( mesh, mShader );

			// Get UV scale and offset.
			ovrHmd_GetRenderScaleAndOffset( eyeFov[eye], toOvr( mRift->mFbo->getSize() ), mRift->mEyeViewport[eye], &mRift->mUVScaleOffset[eye][0] );
		}
	}

	ci::gl::GlslProgRef	mShader;
	ci::gl::BatchRef	mBatch[ovrEye_Count];
};

static const unsigned int kDefaultDistortionCaps =
  ovrDistortionCap_Vignette
| ovrDistortionCap_TimeWarp
| ovrDistortionCap_Overdrive
| ovrDistortionCap_HqDistortion;

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
, mDistortionCaps( kDefaultDistortionCaps )
, mHmdCaps( kDefaultHmdCaps )
, mTrackingCaps( kDefaulTrackingCaps )
, mHmdSettingsChanged( true )
, mIsMonoscopic( false )
, mUsePositionalTracking( true )
, mHmd( nullptr )
{
	mHostCamera.setEyePoint( vec3( 0 ) );
	//TODO: Fix default direction: invert z-axis.
	mHostCamera.setViewDirection( vec3( 0, 0, 1 ) );
	
	if( ovrHmd_Detect() > 0 ) {
		mHmd = ovrHmd_Create( 0 );
		CI_ASSERT( mHmd != nullptr );
		// Set hmd capabilities.
		mHmdCaps = ovrHmd_GetEnabledCaps( mHmd ) | mHmdCaps;
		ovrHmd_SetEnabledCaps( mHmd, mHmdCaps );
	}
	else {
		CI_LOG_E( "Failed to create Hmd." );
//		mHmd = ovrHmd_CreateDebug( ovrHmdType::ovrHmd_DK2 );
	}
}

OculusRift::~OculusRift()
{
	detachFromWindow();
	
	if( mHmd )
		ovrHmd_Destroy( mHmd );
	
	mHmd = nullptr;
}

bool OculusRift::attachToWindow( const ci::app::WindowRef& window )
{
	if( ! mHmd )
		return false;
	
	if( ! isValid( window ) )
		return false;
	
	if( isDesktopExtended() ) {
		mImpl = std::unique_ptr<ExtendedModeImpl>( new ExtendedModeImpl{ this } );
	} else {
		mImpl = std::unique_ptr<DirectModeImpl>( new DirectModeImpl{ this } );
	}
	
	if( ! mImpl->initialize( window ) )
		return false;
	
	// Override the window's startDraw() and finishDraw() methods, so we can inject our own code.
	RendererGlRef rendererGl = std::dynamic_pointer_cast<RendererGl>( window->getRenderer() );
	if( rendererGl ) {
		rendererGl->setStartDrawFn( std::bind( &OculusRift::startDrawFn, this, std::placeholders::_1 ) );
		rendererGl->setFinishDrawFn( std::bind( &OculusRift::finishDrawFn, this, std::placeholders::_1 ) );
	}
	else {
		throw std::runtime_error( "OculusRift can only be used in combination with RendererGl." );
	}
	// Connect to the window close event, so we can properly destroy our HMD.
	window->getSignalClose().connect( std::bind( [&]( ovrHmd hmd ) { assert( hmd == mHmd ); detachFromWindow(); }, mHmd ) );
	
	ovrHmd_ConfigureTracking( mHmd, mTrackingCaps, 0 );
	ovrGLConfig cfg;
	cfg.OGL.Header.API = ovrRenderAPI_OpenGL;
	cfg.OGL.Header.BackBufferSize = mHmd->Resolution;
	cfg.OGL.Header.Multisample = 1; // TODO: support multi-sampling
#if defined( CINDER_MSW )
	cfg.OGL.Window = static_cast<HWND>( window->getNative() );
	cfg.OGL.DC = window->getDc();
#endif
	if( ! ovrHmd_ConfigureRendering( mHmd, &cfg.Config, mDistortionCaps, mHmd->MaxEyeFov, mEyeRenderDesc ) ) {
		throw std::runtime_error( "Failed to configure rendering." );
	}
	updateHmdSettings();
	
	mWindow = window;
	return true;
}

void OculusRift::detachFromWindow()
{
	if( !isValid( mWindow ) )
		return;

	RendererGlRef rendererGl = std::dynamic_pointer_cast<RendererGl>( mWindow->getRenderer() );
	if( rendererGl ) {
		rendererGl->setStartDrawFn( nullptr );
		rendererGl->setFinishDrawFn( nullptr );
	}
	
	mWindow->setPos( ivec2( 0 ) );

	mWindow.reset();
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
#if defined( CINDER_MSW )
	assert( mHmd );
	return ( mHmd->HmdCaps & ovrHmdCap_ExtendDesktop ) != 0;
#else
	return true;
#endif
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

bool OculusRift::isTracked() const
{
	auto status = mTrackingState.StatusFlags;
	bool tracked = ( status & ovrStatus_PositionConnected ) && ( status & ovrStatus_PositionTracked );
	return tracked && isPositionalTrackingEnabled();
}

bool OculusRift::isMirrored() const
{
	return ! ( mHmdCaps & ovrHmdCap_NoMirrorToWindow );
}

void OculusRift::enableMirrored( bool enabled )
{
	if( enabled ) {
		mHmdCaps &= ~ovrHmdCap_NoMirrorToWindow;
	} else {
		mHmdCaps |= ovrHmdCap_NoMirrorToWindow;
	}
	mHmdSettingsChanged = true;
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

void OculusRift::dismissHSW()
{
	ovrHSWDisplayState hswDisplayState;
	ovrHmd_GetHSWDisplayState( mHmd, &hswDisplayState );
	if( hswDisplayState.Displayed ) {
		ovrHmd_DismissHSWDisplay( mHmd );
	}
}

void OculusRift::initializeFrameBuffer()
{
	// Determine the size and create the buffer.
	ovrSizei left = ovrHmd_GetFovTextureSize( mHmd, ovrEye_Left, mHmd->DefaultEyeFov[ovrEye_Left], mScreenPercentage );
	ovrSizei right = ovrHmd_GetFovTextureSize( mHmd, ovrEye_Right, mHmd->DefaultEyeFov[ovrEye_Right], mScreenPercentage );
	int w = left.w + right.w;
	int h = math<int>::max( left.h, right.h );
	
	if( ! mFbo || mFbo->getWidth() != w || mFbo->getHeight() != h ) {
		// Specify the buffer format.
		gl::Fbo::Format fmt;
		fmt.enableDepthBuffer();
		fmt.setSamples( 0 ); // TODO: support multi-sampling
		mFbo = gl::Fbo::create( w, h, fmt );
		
		// The actual size may be different due to hardware limits.
		w = mFbo->getWidth();
		h = mFbo->getHeight();
		
		// Initialize eye rendering information.
		for( ovrEyeType eye = ovrEye_Left; eye < ovrEye_Count; eye = static_cast<ovrEyeType>( eye + 1 ) ) {
			mEyeTexture[eye].OGL.Header.API = ovrRenderAPI_OpenGL;
			mEyeTexture[eye].OGL.Header.TextureSize = toOvr( mFbo->getSize() );
			mEyeTexture[eye].OGL.Header.RenderViewport.Size.w = mEyeViewport[eye].Size.w = w / 2;
			mEyeTexture[eye].OGL.Header.RenderViewport.Size.h = mEyeViewport[eye].Size.h = h;
			mEyeTexture[eye].OGL.Header.RenderViewport.Pos.x = mEyeViewport[eye].Pos.x = eye * ( w + 1 ) / 2;
			mEyeTexture[eye].OGL.Header.RenderViewport.Pos.y = mEyeViewport[eye].Pos.y = 0;
			mEyeTexture[eye].OGL.TexId = mFbo->getColorTexture()->getId();
		}
	}
}

void OculusRift::startDrawFn( Renderer *renderer )
{
	renderer->makeCurrentContext();
	dismissHSW();
	initializeFrameBuffer();

	if( mHmdSettingsChanged ) {
		updateHmdSettings();
	}

	mImpl->beginFrame();

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
	mImpl->endFrame( renderer );
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

 
