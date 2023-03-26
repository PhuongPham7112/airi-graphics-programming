#version 150

in vec3 position;
in vec3 color;

out vec4 col;

uniform mat4 modelViewMatrix;
uniform mat4 projectionMatrix;
uniform int shaderMode;
uniform float maxHeight;

void main()
{
  // compute the transformed and projected vertex position (into gl_Position) 
  // compute the vertex color (into col)
	gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0f);
	col = vec4(color, 1.0f);
}

