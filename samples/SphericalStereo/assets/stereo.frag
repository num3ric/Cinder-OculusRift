#version 150

uniform sampler2D	uTex0;
uniform bool		uTopDown;
uniform bool		uLeftFirst;
uniform bool		uCurrentEye;

in vec3		vModelNormal;
out vec4 	fragColor;

const float PI = 3.14159265359;
const float kOneOverPi = 1.0 / 3.14159265;

void main() {
	vec3 N = normalize( vModelNormal );
	// spherical projection based on the surface normal
	vec2 coord = vec2( 0.5 + 0.5 * atan( N.x, -N.z ) * kOneOverPi, 1.0 - acos( N.y ) * kOneOverPi );
	if( uTopDown ) {
		coord.y = 0.5 * coord.y + float( uCurrentEye != uLeftFirst ) * 0.5;
	} else {
		coord.x = 0.5 * coord.x + float( uCurrentEye != uLeftFirst ) * 0.5;
	}
	fragColor = texture( uTex0, coord );
}