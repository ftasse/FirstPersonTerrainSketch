#version 130

in vec3 position;
uniform sampler2D terrain;
uniform mat4 mvp;

out vec2 texcoord;
out vec3 pos;

void main(void){
    texcoord = position.xy;
    float height = texture(terrain, texcoord).a;
    gl_Position = mvp * vec4(position.x, position.y, height, 1.0);
	pos = position;
}
