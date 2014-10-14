#version 130

in vec3 position;
uniform sampler2D terrain;
uniform mat4 mvp;

out vec3 color;
out vec2 texcoord;

const vec2 size = vec2(2.0,0.0);
const ivec3 off = ivec3(-1,0,1);

vec3 getNormal(vec2 tex_coord)
{
    float s11 = texture(terrain, tex_coord).r;
    float s01 = textureOffset(terrain, tex_coord, off.xy).r;
    float s21 = textureOffset(terrain, tex_coord, off.zy).r;
    float s10 = textureOffset(terrain, tex_coord, off.yx).r;
    float s12 = textureOffset(terrain, tex_coord, off.yz).r;
    vec3 va = normalize(vec3(size.xy,s21-s01));
    vec3 vb = normalize(vec3(size.yx,s12-s10));
    vec4 bump = vec4( cross(va,vb), s11 );
    return bump.xyz;
}

void main()
{
    texcoord = vec2(position.x, position.y);
    vec3 normal = normalize(getNormal(texcoord));
    color = vec3(normal);

    gl_Position = mvp*vec4(position, 1.0);
}
