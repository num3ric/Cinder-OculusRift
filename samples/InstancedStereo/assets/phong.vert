#version 410

uniform mat4 ciModelMatrix;
uniform mat4 uWorldToEyeClipMatrices[6];

in vec4 ciPosition;
in vec3 ciNormal;

out vec4 vViewPosition;
out vec3 vNormal;
out mat4 vEyeViewMatrix;
out float gl_ClipDistance[1];

vec4 eyeClipEdge[2] = vec4[2]( vec4(-1,0,0,0), vec4(1,0,0,0) );
float eyeOffsetScale[2] = float[2]( -0.5, 0.5 );

void main( void )
{
	uint eyeIndex = gl_InstanceID;
	vEyeViewMatrix = uWorldToEyeClipMatrices[ 3 * eyeIndex ];
	mat4 eyeProjectionMatrix = uWorldToEyeClipMatrices[ 3 * eyeIndex + 1 ];
	mat3 eyeNormalMatrix = mat3( uWorldToEyeClipMatrices[ 3 * eyeIndex + 2 ] );

	vec4 clipPos;
	vViewPosition = vEyeViewMatrix * ciModelMatrix * ciPosition;
	vNormal = mat3(vEyeViewMatrix) * eyeNormalMatrix * ciNormal;
	clipPos = eyeProjectionMatrix * vViewPosition;
	clipPos.x *= 0.5; // shrink to half of the screen
	clipPos.x += eyeOffsetScale[eyeIndex] * clipPos.w; // scoot left or right.

	gl_Position = clipPos;
	gl_ClipDistance[0] = dot( eyeClipEdge[eyeIndex], clipPos );
}