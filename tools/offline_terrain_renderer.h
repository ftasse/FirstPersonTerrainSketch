#ifndef OFFLINE_TERRAIN_RENDERER_H
#define OFFLINE_TERRAIN_RENDERER_H

#include "tools/offline_renderer.h"
#include "geometry/terrain.h"

class OfflineTerrainRenderer : public OfflineRenderer
{
public:
    explicit OfflineTerrainRenderer(Terrain *terrain);
    ~OfflineTerrainRenderer();

    void unProject(std::vector<Eigen::Vector2i> &window_points,
                   std::vector<Eigen::Vector3f> &unprojected_points);
protected:
    Terrain *terrain_;

    Eigen::Vector3f getFaceCentroid(int i, int j);
    void drawTerrain();
};

#endif // OFFLINE_TERRAIN_RENDERER_H
