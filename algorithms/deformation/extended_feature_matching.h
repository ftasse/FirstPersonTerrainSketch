#ifndef EXTENDED_FEATURE_DEFORMATION_H
#define EXTENDED_FEATURE_DEFORMATION_H

#include "algorithms/deformation/feature_matching.h"
#include <set>

class ExtendedFeatureMatching : public FeatureMatching
{
public:
    ExtendedFeatureMatching(Terrain *terrain, std::vector< Sketch *> &sketches,
                            std::vector<  std::vector<Terrain::SilhouetteSegment > > &silhouettes);

    virtual void computeAssignments();
    virtual void computePossibleCutsForStroke(int sketch_id, int stroke_id, std::vector<PolylineCut> &polyline_cuts);
    void computeCutsCosts(int sketch_id, int stroke_id, std::vector<PolylineCut> &polyline_cuts);

private:
    struct Polygon
    {
    private:
        enum PolySide {kNone, kLeft, kRight};

        PolySide getSide(Eigen::Vector3f a, Eigen::Vector3f b)
        {
            float x = (a[0]*b[1] - a[1]*b[0]);
            if (x < 1e-6)  return kLeft;
            else if (x > 1e-6)  return kRight;
            else return kNone;
        }

    public:
        std::vector<Eigen::Vector3f> points;
        std::set< std::pair<int, int> > unavailable_positions;
        int sketch_id;

        Polygon(){ sketch_id = -1;}

        bool isInside(Eigen::Vector3f point) { //Assume convex polygon
            int n_vertices = points.size();
            PolySide previous_side = kNone;
            point[2] = 0;
            for (int i = 0; i< n_vertices; ++i)
            {
                Eigen::Vector3f a = points[i], b = points[(i+1)%n_vertices];
                Eigen::Vector3f affine_segment = b-a, affine_point = point - a;
                PolySide current_side = getSide(affine_segment, affine_point);
                if (current_side  == kNone)
                    return false; //outside or over an edge
                else if (previous_side  == kNone) //first segment
                    previous_side = current_side;
                else if (previous_side != current_side)
                    return false;
            }
            return true;
        }

        bool isInside(PolylineCut &cut)
        {
            for (unsigned int k=0; k<cut.deformed_points.size(); ++k)
            {
                if (isInside(cut.deformed_points[k]))
                    return true;

                std::pair<int, int> pos ((int)cut.deformed_points[k][0], (int)cut.deformed_points[k][1]);
                if (unavailable_positions.find(pos) != unavailable_positions.end())
                    return true;
            }
            return false;
        }

        bool intersects(const Polygon &polygon)
        {
          for (int i  = 0; i < ((int)points.size()) - 1; ++i)
          {
              for (int j  = 0; j < ((int)polygon.points.size()) - 1; ++j)
              {
                  Eigen::Vector3f p1 = points[i]; Eigen::Vector3f p2 = points[i+1];
                  Eigen::Vector3f q1 = polygon.points[j]; Eigen::Vector3f q2 = polygon.points[j+1];

                  p1[2] = p2[2] = q1[2] = q2[2] = 0.0;

                  float d = (p1[0]-p2[0])*(q1[1]-q2[1]) - (p1[1]-p2[1])*(q1[0]-q2[0]);
                  if (d == 0) continue;

                  float xi = ((q1[0]-q2[0])*(p1[0]*p2[1]-p1[1]*p2[0])-(p1[0]-p2[0])*(q1[0]*q2[1]-q1[1]*q2[0]))/d;
                  float yi = ((q1[1]-q2[1])*(p1[0]*p2[1]-p1[1]*p2[0])-(p1[1]-p2[1])*(q1[0]*q2[1]-q1[1]*q2[0]))/d;

                  if (xi < std::min(p1[0], p2[0]) || xi >std::max(p1[0], p2[0])) continue;
                  if (xi < std::min(q1[0], q2[0]) || xi > std::max(q1[0], q2[0])) continue;
                  return true;
              }
          }

          return false;
        }
    };

    std::vector<Polygon> unavailable_polygon_regions;

    bool isCutAvailable(PolylineCut &cut);
    bool isCutAvailable(PolylineCut &cut, std::vector<Polygon> &regions);
    void updateAvailabilityFrom(PolylineCut &cut, bool front_area);
    void buildPolygonFrom(PolylineCut &cut, bool front_area, Polygon &polygon);


    float branch(std::vector< std::vector< std::vector<PolylineCut> > > &cuts_per_stroke,
           std::vector<Polygon> &polygon_regions,
           std::vector< std::vector< std::vector<PolylineCut> > > &cur_deformed_features_per_sketch,
           float &cur_best_cost,
           float cur_cost = 0,
           int cur_sketch_id = 0,
           int cur_stroke_id = 0,
           int cur_cut_id = 0);
};

#endif // EXTENDED_FEATURE_DEFORMATION_H
