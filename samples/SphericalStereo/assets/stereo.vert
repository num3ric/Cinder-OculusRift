#version 150

uniform mat4	ciModelViewProjection;
uniform mat3	ciNormalMatrix;
in vec4			ciPosition;
in vec3			ciNormal;

out vec3		vModelNormal;

void main( void )
{
	vModelNormal = ciNormal;
	gl_Position = ciModelViewProjection * ciPosition;
}
