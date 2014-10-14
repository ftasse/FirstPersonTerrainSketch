#ifndef DEFORMATION_H
#define DEFORMATION_H

#include "geometry/terrain.h"
#include "geometry/sketch.h"

class Deformation
{
public:
    Deformation(Terrain *terrain, Sketch *sketch);

    double update();
    void updateDiffuseTexture();

protected:
    Terrain *terrain_;
    Sketch *sketch_;
};

double deformTerrainWithConstraints(
    Terrain *terrain,
    const std::vector<CameraInfo> &camera_infos,
    const std::vector<Terrain::SilhouetteSegment> &constraints);

#endif // DEFORMATION_H
