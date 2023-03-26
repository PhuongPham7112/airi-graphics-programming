#version 150

in vec3 position;
in vec3 color;

out vec3 viewPosition;
out vec3 viewNormal;

uniform mat4 modelViewMatrix;
uniform mat4 projectionMatrix;
uniform mat4 normalMatrix;

void main()
{
	vec4 viewPosition4 = modelViewMatrix * vec4(position, 1.0f);
	viewPosition = viewPosition4.xyz;

	gl_Position = projectionMatrix * viewPosition4;
	viewNormal = normalize((normalMatrix*vec4(color, 0.0f)).xyz);
}

