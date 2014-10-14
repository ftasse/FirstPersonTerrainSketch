#ifndef TERRAIN_RENDERER_H
#define TERRAIN_RENDERER_H

#include <vector>
#include "geometry/terrain.h"

class TerrainFastRenderer
{
public:
    TerrainFastRenderer();
    void init();
    void cleanup();
    void display();

    Terrain* getTerrain() const;
    void setTerrain(Terrain *terrain);
    void updateTerrain();
    
    void setAccurateNormals(bool status) { accurate_normals_ = status; }

private:
    Terrain *terrain_;
    unsigned int heightmap_texid_, vbo_, vao_;
    std::vector<unsigned int> shaders_;
    unsigned int shader_program_;
    bool accurate_normals_;

    bool gl4xSupported_;

    int position_location_;
    int mvp_location_, heightmap_location_;
    int screensize_location_, lod_factor_location_;

    int width() { return (terrain_!=NULL)?terrain_->width():0; }
    int height() { return (terrain_!=NULL)?terrain_->height():0; }

    void updateHeightmapTexture(); //GL_RGBA32F, R is the elevtion and GBA the elevation
    void computeNormals(unsigned int elevation_texid);
    
    void setupTerrainVertexPositions(const int quad_with, const int quad_height);
    
    // Detail textures
    unsigned int details_texids[6];
    void initDetailTextures();
    unsigned int diffuse_texid;
    void initDiffuseTexture();

public:
    int num_xquads;
    int num_yquads;
};

#endif // TERRAIN_RENDERER_H
