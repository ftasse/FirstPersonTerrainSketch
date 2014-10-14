#version 400

in vec3 position;
uniform sampler2D terrain;
uniform mat4 mvp;

out vec2 texcoord;

void main(void){
    texcoord = position.xy;
    float height = texture(terrain, texcoord).a;
    gl_Position = vec4(position.x, position.y, height, 1.0);    //if using tes and tcs
}
