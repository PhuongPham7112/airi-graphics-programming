#version 150

in vec3 position;
in vec3 position1;
in vec3 position2;
in vec3 position3;
in vec3 position4;
in vec4 color;
out vec4 col;

uniform mat4 modelViewMatrix;
uniform mat4 projectionMatrix;
uniform int shaderMode;

void main()
{
  // compute the transformed and projected vertex position (into gl_Position) 
  // compute the vertex color (into col)
  if (shaderMode == 1)
  {
	vec3 smoothPos = (position1 + position2 + position3 + position4) / 4;
	gl_Position = projectionMatrix * modelViewMatrix * vec4(smoothPos, 1.0f);
	col = color;
  }
  else 
  {
	gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0f);
	col = color;
  }
}

