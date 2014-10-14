#ifndef FEATURE_DETECTION_H
#define FEATURE_DETECTION_H

#include "geometry/terrain.h"
#include "camera_info.h"

class FeatureDetection
{
public:
    FeatureDetection(Terrain *terrain);

    void computeRidges(const CameraInfo &camera_info,
                       std::vector<Terrain::SilhouetteSegment> &silsegments);

private:
    Terrain *terrain_;
};

#endif // FEATURE_DETECTION_H
