#version 150

uniform sampler2D	uTex0;
uniform ivec2		uDisplayMode; // LR, RL, TB, BT
uniform int			uRightEye;

in vec3		vModelNormal;
out vec4 	fragColor;

const float ONE_OVER_PI = 1.0 / 3.14159265;

void main() {
	vec3 N = normalize( vModelNormal );
	// spherical projection based on the surface normal
	vec2 coord = vec2( 0.5 + 0.5 * atan( N.x, -N.z ) * ONE_OVER_PI, 1.0 - acos( N.y ) * ONE_OVER_PI );
	if( bool( uDisplayMode.x ) ) {
		coord.y = 0.5 * coord.y + float( uRightEye != uDisplayMode.y ) * 0.5;
	} else {
		coord.x = 0.5 * coord.x + float( uRightEye == uDisplayMode.y ) * 0.5;
	}
	fragColor = texture( uTex0, coord );
}