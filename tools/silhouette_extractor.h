#ifndef SILHOUETTE_EXTRACTION_H
#define SILHOUETTE_EXTRACTION_H

#include "tools/offline_terrain_renderer.h"
#include "tools/ppa/dtts_ppa.h"

//#include "opencv2/core/core.hpp"

class SilhouetteExtractor : public OfflineTerrainRenderer
{
    enum FacingMode {kFrontFacing, kBackFacing, kInvisible};

public:
    explicit SilhouetteExtractor(Terrain *terrain);
    ~SilhouetteExtractor();

    void compute(const CameraInfo &camera_info,
                 std::vector<Terrain::SilhouetteSegment>  &silhouette_segments);
    void compute_naive(std::vector<Terrain::SilhouetteSegment>  &silhouette_segments);
    void compute_fast(std::vector<Terrain::SilhouetteSegment>  &silhouette_segments);

//    void computeFromContour(std::vector<Terrain::SilhouetteSegment> &silhouette_segments,
//                            cv::Mat *img);

    void computeVisibleFrontFacingStatus();
    bool isFrontFace(int i, int j);
    bool isBackFace(int i, int j);

private:
    FacingMode *front_facing_ ;
    FacingMode *front_facing_per_tri_ ;

    bool checkSilhouetteEdge(int i1, int j1, int i2, int j2);
    FacingMode checkTriangleFrontFacing(int i1, int j1, int i2, int j2, int i3, int j3);
    void computeTriangleFrontFacingStatus();

    bool checkVisibility(Eigen::Vector3f point);
    void drawTerrainWireFrame(float precision = 0.5);

    void reconnectSilhouetteSegments(CameraInfo camera_info,
                                     std::vector<Terrain::SilhouetteSegment>  &silhouette_segments);
};

void computeTreeFromSilhouetteSegments(std::vector<Terrain::SilhouetteSegment>  &silhouette_segments,
                                       Dtts::Tree *tree);

#endif // SILHOUETTE_EXTRACTION_H
