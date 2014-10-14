#version 400

layout(quads, equal_spacing, ccw) in;
out vec2 texcoord;
out vec3 pos;
out float depth;

uniform sampler2D terrain;
uniform sampler2D perlin;
uniform mat4 mvp;



//gl_TessCoord is the normal coordinate position of a vertex
//in a tesselated abstract patch

float slopeCoeff(float slope)
{
//        float slope = 1.0-dot(normal, vec3(0,0,1));
        float rockSlope = 0.95;
        float rockTransition = 0.1;
        float rockInterpCoeff = (slope-(rockSlope)) / rockTransition;
        return clamp(rockInterpCoeff, 0, 1);
}

void main(){
    float u = gl_TessCoord.x;
    float v = gl_TessCoord.y;

    //Interpolation to get real vertex position
    vec4 a = mix(gl_in[1].gl_Position, gl_in[0].gl_Position, u);
    vec4 b = mix(gl_in[2].gl_Position, gl_in[3].gl_Position, u);
    vec4 position = mix(a, b, v);
    pos = position.xyz / position.w;
    texcoord = position.xy;
    

    float height = texture(terrain, texcoord).a;
    gl_Position = mvp * vec4(vec3(texcoord, height), 1.0);
    depth = gl_Position.z;
}
