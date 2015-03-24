#version 410

uniform vec4 uLightPosition;

in vec4 vViewPosition;
in vec3 vNormal;
in mat4 vEyeViewMatrix;

out vec4 oColor;

void main( void )
{		
	const vec3 matDiffuse = vec3( 1 );
	const vec3 lightDiffuse = vec3( 1 );

	vec3 N = normalize( vNormal );
	vec3 L = normalize( vec3( vEyeViewMatrix * uLightPosition - vViewPosition ) );

	// hemispherical ambient lighting
	float hemi = dot( N, vec3(0,1,0) ) * 0.5 + 0.5;
	oColor.rgb = mix( vec3(0), vec3(0.1), hemi );

	// diffuse lighting
	float lambert = max( 0, dot( N, L ) );
	oColor.rgb += lambert * matDiffuse * lightDiffuse;

	// output gamma corrected color
	oColor.rgb = sqrt( oColor.rgb );
	oColor.a = 1;
}