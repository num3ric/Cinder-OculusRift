#version 150

uniform sampler2D	uTex0;
uniform bool		uTopDown;
uniform bool		uLeftFirst;
uniform bool		uCurrentEye;

in vec3		vModelNormal;
out vec4 	fragColor;

const float PI = 3.14159265359;

void main() {
	vec3 N = normalize( vModelNormal );
	// spherical projection based on the surface normal
	vec2 coord = vec2( 0.5 ) + vec2( 0.5 * atan( N.z, N.x ), atan( N.y, length( N.xz ) ) ) / PI;
	coord.x = mod( coord.x - 0.25, 1.0);

	if( uTopDown ) {
		coord.y = 0.5 * coord.y + float( uCurrentEye == uLeftFirst ) * 0.5;
	} else {
		coord.x = 0.5 * coord.x + float( uCurrentEye != uLeftFirst ) * 0.5;
	}
	fragColor = texture( uTex0, coord );
}