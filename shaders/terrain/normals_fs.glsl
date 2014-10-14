#version 130

in vec3 color;
uniform sampler2D terrain;
in vec2 texcoord;

out vec4 fragment;

void main()
{
    fragment = vec4(color, 5); //texture(terrain, texcoord).r);
}
