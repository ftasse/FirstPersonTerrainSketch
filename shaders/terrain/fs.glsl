#version 130

in vec3 pos;
in vec2 texcoord;

uniform sampler2D terrain;

uniform sampler2D grass;
uniform sampler2D grass2;
uniform sampler2D ground;
uniform sampler2D perlin;
uniform sampler2D rock;
uniform sampler2D snow;

uniform float maxHeight;


out vec4 fragment;

const vec2 size = vec2(2.0,0.0);
const ivec3 off = ivec3(-1,0,1);

float slopeCoeff(vec3 normal)
{
        return 1.0;
        float slope = 1.0 - abs(dot(normal, vec3(0,0,1)));
        return slope*slope;
        float rockSlope = 0.25;
        float rockTransition = 0.01;
        float rockInterpCoeff = (slope-(rockSlope)) / rockTransition;
        return clamp(rockInterpCoeff, 0, 1);
}

vec3 getNormal(vec2 tex_coord)
{
    float s11 = texture(terrain, tex_coord).a;
    float s01 = textureOffset(terrain, tex_coord, off.xy).a;
    float s21 = textureOffset(terrain, tex_coord, off.zy).a;
    float s10 = textureOffset(terrain, tex_coord, off.yx).a;
    float s12 = textureOffset(terrain, tex_coord, off.yz).a;
    vec3 va = normalize(vec3(size.xy,s21-s01));
    vec3 vb = normalize(vec3(size.yx,s12-s10));
    vec4 bump = vec4( cross(va,vb), s11 );
    return bump.xyz;
}

vec3 fractalNormal(vec2 texcoord)
{

    
    ////////////////////////////////////////////////////////////////////////////
    // FRACTAL NOISE
    ////////////////////////////////////////////////////////////////////////////
    
	// COEFFICIENT
	float K = 30.0;
	float L = 30.0;
    
        // NORMAL COMPUTATION
        vec3 normal;
        
        // height
	float terrain_max_height = 1.0;
	float height00  = terrain_max_height*texture(terrain, texcoord).a;
	float height10  = terrain_max_height*textureOffset(terrain, texcoord, ivec2(1,0)*1).a;
	float height20  = terrain_max_height*textureOffset(terrain, texcoord, ivec2(2,0)*1).a;
	float height01  = terrain_max_height*textureOffset(terrain, texcoord, ivec2(0,1)*1).a;
	float height02  = terrain_max_height*textureOffset(terrain, texcoord, ivec2(0,2)*1).a;
	float height11  = terrain_max_height*textureOffset(terrain, texcoord, ivec2(1,1)*1).a;
	
	
        vec3 va00 = normalize(vec3(1,0,height10-height00));
        vec3 vb00 = normalize(vec3(0,1,height01-height00));
        vec3 bump00 = normalize(cross(va00,vb00));
        
        vec3 va10 = normalize(vec3(1,0,height20-height10));
        vec3 vb10 = normalize(vec3(0,1,height11-height10));
        vec3 bump10 = normalize(cross(va10,vb10));
        
        vec3 va01 = normalize(vec3(1,0,height11-height01));
        vec3 vb01 = normalize(vec3(0,1,height02-height01));
        vec3 bump01 = normalize(cross(va01,vb01));
	
	float slope00 = slopeCoeff(bump00);
	float slope10 = slopeCoeff(bump10);
	float slope01 = slopeCoeff(bump01);

	
	// noise
	float noise00 = abs(texture(perlin, texcoord*20.0).r);
	float noise10 = abs(textureOffset(perlin, texcoord*20.0, ivec2(1,0)).r);
	float noise01 = abs(textureOffset(perlin, texcoord*20.0, ivec2(0,1)).r);
	
	// Elevation derivative
	float dElevDx = height20 - height00;
	float dElevDz = height02 - height00;
	
	// Slope derivative
	float dSlopeDx = slope10 - slope00;
	float dSlopeDz = slope01 - slope00;
	
	// Noise 
	float dNoiseDx = noise10 - noise00;
	float dNoiseDz = noise01 - noise00;
	
	// TANGENT SPACE
	// u = JACOBIAN * (-1,0,1)
	// v = JACOBIAN * (1,0,1)
	vec3 uj,vj;
	
	uj.x = - 1 - K * (slope00 * dNoiseDx + noise00 * dSlopeDx) + K * (slope00 * dNoiseDz + noise00 * dSlopeDz);
	uj.y = - dElevDx + dElevDz;
	uj.z = - L * (slope00 * dNoiseDx + noise00 * dSlopeDx) + 1 + L * (slope00 * dNoiseDz + noise00 * dSlopeDz);
	
	vj.x = + 1 + K * (slope00 * dNoiseDx + noise00 * dSlopeDx) + K * (slope00 * dNoiseDz + noise00 * dSlopeDz);
	vj.y = + dElevDx + dElevDz;
	vj.z = + L * (slope00 * dNoiseDx + noise00 * dSlopeDx) + 1 + L * (slope00 * dNoiseDz + noise00 * dSlopeDz);
	

	
	normal = normalize(cross(uj, vj));
	float swap = normal.z;
	normal.z = normal.y;
	normal.y = swap;
	
	return normal;
}


/* int isoValue(vec2 coords, ivec2 offset, float scale, float step)
{
        return int ( textureLodOffset(terrain, coords, scale, offset).a / step ) ;
} */

bool isIsoLine(vec2 coords, float scale, float step)
{
        int iso00 = int(textureLodOffset(terrain, coords, scale, ivec2(0,0)).a / step); //isoValue(coords, ivec2(0,0), scale, step);
        int iso01 = int(textureLodOffset(terrain, coords, scale, ivec2(0,1)).a / step); //isoValue(coords, ivec2(0,1), scale, step);
        int iso10 = int(textureLodOffset(terrain, coords, scale, ivec2(1,0)).a / step); //isoValue(coords, ivec2(1,0), scale, step);
        
        if ( (iso00 - iso01) != 0  || (iso00-iso10) != 0)
        {
                return true;
        }
        else
                return false;
}






void main()
{
//    vec3 normal = getNormal(texcoord); // = texture(terrain, texcoord).rgb;
    vec3 normal = fractalNormal(texcoord);
    
    
    fragment = vec4(normal, 1.0);
    
    ////////////////////////////////////////////////////////////////////////////
    // Terrain procedural color
    ////////////////////////////////////////////////////////////////////////////
    
    vec3 color;
    vec3 grassColor  = texture(grass, texcoord*10.0).rgb * vec3(1.25,1.50,1.25);
    vec3 groundColor = texture(ground, texcoord*20.0).rgb;
    vec3 snowColor  = texture(snow, texcoord*15.0).rgb;
    vec3 rockColor  = texture(rock, texcoord*30.0).rgb;
    vec3 grass2Color  = texture(grass2, texcoord*40.0).rgb;
    
    // Choose isolines or details
    int useIso = 0; 
    if (useIso == 1)
    {
            
                if ( isIsoLine(texcoord, 0.0, 10) )
                {
                        color.rgb = vec3(0.3,0.3,0.7);
                }
//                else if ( isIsoLine(texcoord, 0.1, 2.0) )
//                {
//                        color.rgb = vec3(0.8,0.8,0.8);
//                }
                else
                {
                        color.rgb = vec3(1.0);
                }
    }
    else
    {
            // Choose between texture and color
            int useTextures = 1;
            if (useTextures == 0)
            {
                grassColor  = vec3(0.1,0.4,0.2);
                grass2Color  = vec3(0.2,0.0,1.0);
                groundColor = vec3(0.4,0.35,0.1);
                snowColor   = vec3(1.0,1.0,1.0);
                rockColor   = vec3(0.6,0.6,0.6);
            } 
            
            // Grass2/Grass
            float perlin0 = texture(perlin, texcoord*3.0).r;
            float grassInterpCoeff = perlin0*perlin0;
            color = mix(grassColor, grass2Color, grassInterpCoeff);
            
            // Grass/Ground
            float perlin1 = texture(perlin, texcoord*1.0).r;
            float groundInterpCoeff = perlin1*perlin1;
            color = mix(color, groundColor, groundInterpCoeff);
            
            // Snow
            float height = pos.z + texture(perlin, texcoord*10.0).r * 10.0;
            float snowHeight = 100.0;//maxHeight*0.85;
            float snowTransition = 10.0;
            float snowInterpCoeff = (height-(snowHeight)) / snowTransition;
            snowInterpCoeff = clamp(snowInterpCoeff, 0, 1);
            color = mix(color, snowColor, snowInterpCoeff);
            
            // Rock
            float slopeNoise = texture(perlin, texcoord * 100.0).r;
            float slope = 1.0 - dot(normal, vec3(0,0,1)) - cos(slopeNoise)*cos(slopeNoise) * 0.05;
            float rockSlope = 0.30;
            float rockTransition = 0.1;
            float rockInterpCoeff = (slope-(rockSlope)) / rockTransition;
            rockInterpCoeff = clamp(rockInterpCoeff, 0, 1);
            color = mix(color, rockColor, rockInterpCoeff);
    }
    
    
    
    // Lambertian
    float shadow = dot(normal, vec3(1,1,1));
//    float shadow = dot(normal, vec3(1,1,1));
    color*= clamp(shadow * 0.4 + 0.6, 0, 1);
    
//    color = fractalNormal(texcoord) * 0.5 + 0.5;
    // Final color
    fragment.rgb = color;   
}
