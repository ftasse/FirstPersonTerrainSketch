#include "feature_detection.h"

#include "tools/ppa/dtts_ppa.h"

void computeBranches(Dtts::Tree &ridges, std::vector<node_list> &branches);

FeatureDetection::FeatureDetection(Terrain *terrain):
    terrain_(terrain)
{
}

void FeatureDetection::computeRidges(const CameraInfo &camera_info,
                                     std::vector<Terrain::SilhouetteSegment> &silsegments)
{
    silsegments.clear();

    Dtts::Tree ridges;
    ridges.runPPA(*terrain_);

    std::vector<node_list> branches;
    Dtts::computeBranches(ridges, branches);

    for (unsigned int i = 0; i < branches.size(); ++i)
    {
        const node_list& branch = branches[i];
        if (branch.size() < 3)  continue;

        Terrain::SilhouetteSegment seg;
        for (node_list::const_iterator it3 = branch.begin(); it3!=branch.end(); it3++)
        {
           node_t b = *it3;
           Eigen::Vector3f point(b.x, b.y, terrain_->getAltitude(b.x, b.y));

           Eigen::Vector3f projector = (point - camera_info.position).normalized();
           float theta = acos(camera_info.direction.normalized().dot(projector));
           if (theta > camera_info.fov_in_rads)
           {
               if (seg.points.size() >=3)  silsegments.push_back(seg);
               seg.points.clear();
           } else
                seg.points.push_back(point);
        }
        if (seg.points.size() >=3)  silsegments.push_back(seg);
    }

}
