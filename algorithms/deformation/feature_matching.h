#ifndef FEATURE_DEFORMATION_H
#define FEATURE_DEFORMATION_H

#include "tools/deformation_weighting.h"
#include "geometry/terrain.h"
#include "geometry/sketch.h"

#include <map>
#include <vector>

#include <Eigen/Core>

struct PolylineCut
{
    int sketch_id;

    int polyline_id;
    int left_pos;
    int right_pos;
    float longest_edge_length;
    float furthest_dist_to_eye;
    float closest_dist_to_eye;

    int id;

    std::pair<int, int> stroke_interval;
    float cost;
    int nb_extruded;

    std::vector<Eigen::Vector3f> deformed_points;

    PolylineCut(int _polyline_id=-1, int _left_pos = -1, int _right_pos = -1):
        polyline_id(_polyline_id),left_pos(_left_pos), right_pos(_right_pos),
        longest_edge_length(0), furthest_dist_to_eye(0), closest_dist_to_eye(0)
    {
      sketch_id = -1;
    }

    bool operator ==(const PolylineCut &rhs) const
    {
        return polyline_id == rhs.polyline_id && left_pos == rhs.left_pos && right_pos == rhs.right_pos;
    }

    bool operator <(const PolylineCut &rhs) const
    {
        if (polyline_id != rhs.polyline_id)
            return polyline_id < rhs.polyline_id;
        else if (left_pos != rhs.left_pos)
            return left_pos < rhs.left_pos;
        else
            return right_pos < rhs.right_pos;
    }
};

/**
  Here, silhouette segments are in fact features
  **/
class FeatureMatching
{
    typedef PolylineCut SketchCut;
    typedef std::map<SketchCut, PolylineCut> Assignment;


public:
    FeatureMatching(Terrain *terrain, std::vector<Sketch*> &sketches,
                    std::vector< std::vector<Terrain::SilhouetteSegment> > &silhouettes);

    //inherited
    void update(DeformationWeighting weighting);

    std::vector< std::vector<Terrain::SilhouetteSegment> > getProjectedFeatures()
    { return projected_features_; }

    std::vector< std::vector<Terrain::SilhouetteSegment> > getFeatures()
    {  return features_; }

    std::vector<Terrain::SilhouetteSegment> getDeformedFeatures()
    { return deformed_features_; }

    void setDeformedFeatures(std::vector<Terrain::SilhouetteSegment> &feat)
    { deformed_features_ = feat; }

protected:
    Terrain *terrain_;
    std::vector<Sketch*> sketches_;

    std::vector< std::vector< std::vector<PolylineCut> > > deformed_features_per_sketch;
    std::vector<Terrain::SilhouetteSegment> deformed_features_;

    DeformationWeighting weighting_;

    virtual void computeAssignments();

    void computePossibleCutsForStroke(int sketch_id, int stroke_id,
                                      std::vector<PolylineCut> &polyline_cuts);

    Eigen::Vector3f projectOnSketchPlane(int sketch_id, Eigen::Vector3f point);

private:
    std::vector< std::vector<Terrain::SilhouetteSegment> > features_;
    std::vector< std::vector<Terrain::SilhouetteSegment> > projected_features_;
    std::vector< std::vector<Terrain::SilhouetteSegment> > projected_sketches_;



    void splitOndulatingFeatures();
    void computeProjections();
    void computeDeformedFeatures();


};

bool lessPolylineCut(const PolylineCut &c1, const PolylineCut &c2);

#endif // FEATURE_DEFORMATION_H
