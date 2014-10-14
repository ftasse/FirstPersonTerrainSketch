#version 400

layout(vertices = 4) out;

uniform vec2 screen_size;
uniform mat4 mvp;
uniform float lod_factor;

//The minimun for the max tesselation level is 64

bool offscreen(vec4 vertex){
    if(vertex.z < -0.5){
        return true;
    }

	if (vertex.x < -1.7 || vertex.y < -1.7 || vertex.x > 1.7 || vertex.y > 1.7)
		return true;
    else
	   return false;
       
    /*return any(
        lessThan(vertex.xy, vec2(-1.7)) ||
        greaterThan(vertex.xy, vec2(1.7))
    );*/
}

//project a world space vertex to device normal space
vec4 project(vec4 vertex){
    vec4 result = mvp * vertex;
    result /= result.w;
    return result;
}

//converts a device normal space vector to screen space
vec2 screen_space(vec4 vertex){
    return (clamp(vertex.xy, -1.3, 1.3)+1) * (screen_size*0.5);
}

float level(vec2 v0, vec2 v1){
    return clamp(distance(v0, v1)/lod_factor, 1, 64);
}

//gl_InvocationID is the index of the TCS invocation within this patch
//gl_TessLevelInner and gl_TessLevelOuter are path variables

void main(){
    if(gl_InvocationID == 0) {
        vec4 v0 = project(gl_in[0].gl_Position);
        vec4 v1 = project(gl_in[1].gl_Position);
        vec4 v2 = project(gl_in[2].gl_Position);
        vec4 v3 = project(gl_in[3].gl_Position);

        if(all(bvec4(offscreen(v0), offscreen(v1), offscreen(v2), offscreen(v3)))){
            gl_TessLevelInner[0] = 0;
            gl_TessLevelInner[1] = 0;
            gl_TessLevelOuter[0] = 0;
            gl_TessLevelOuter[1] = 0;
            gl_TessLevelOuter[2] = 0;
            gl_TessLevelOuter[3] = 0;
        }
        else{
            vec2 ss0 = screen_space(v0);
            vec2 ss1 = screen_space(v1);
            vec2 ss2 = screen_space(v2);
            vec2 ss3 = screen_space(v3);

             float e0 = level(ss1, ss2) * 10.0;
             float e1 = level(ss0, ss1) * 10.0;
             float e2 = level(ss3, ss0) * 10.0;
             float e3 = level(ss2, ss3) * 10.0;

            gl_TessLevelInner[0] = mix(e1, e3, 0.5);
            gl_TessLevelInner[1] = mix(e0, e2, 0.5);
            gl_TessLevelOuter[0] = e0;
            gl_TessLevelOuter[1] = e1;
            gl_TessLevelOuter[2] = e2;
            gl_TessLevelOuter[3] = e3;
        }
    }
    gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
}
