#ifndef PROTUDED_SILHOUETTES_MATCHING_H
#define PROTUDED_SILHOUETTES_MATCHING_H

#include "geometry/terrain.h"
#include "geometry/sketch.h"

#include <map>
#include <vector>

#include <Eigen/Core>


class ProtrudedSilhouetteMatching
{

public:
    ProtrudedSilhouetteMatching(Terrain *terrain, Sketch* sketch);

    std::vector<Terrain::SilhouetteSegment> getDeformedFeatures()
    { return deformed_features_; }

    void setDeformedFeatures(std::vector<Terrain::SilhouetteSegment> &feat)
    { deformed_features_ = feat; }

    std::vector<Terrain::SilhouetteSegment> getUnallocatedSilhouettes()
    {
        if (unallocated_silhouettes_.size() == 0)
            computeUnallocatedSilhouettes();
        return unallocated_silhouettes_;
    }

    void updateSilhouettes();

    int numUnallocatedSilhouettes()
    {
      return unallocated_silhouettes_.size();
    }

protected:
    Terrain *terrain_;
    Sketch *sketch_;

    std::vector<Terrain::SilhouetteSegment> unallocated_silhouettes_;
    std::vector<Terrain::SilhouetteSegment> deformed_features_;


    virtual void computeUnallocatedSilhouettes();

    Eigen::Vector3f projectOnSketchPlane(Eigen::Vector3f point);

private:

};

#endif // PROTUDED_SILHOUETTES_MATCHING_H
