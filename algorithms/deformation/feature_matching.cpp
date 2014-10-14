#include "feature_matching.h"
#include "tools/silhouette_extractor.h"

#include <float.h>
#include <algorithm>
#include <Eigen/Geometry>


bool lessPolylineCut(const PolylineCut &c1, const PolylineCut &c2)
{
    return c1.cost < c2.cost;
}

struct Cone
{
    float side_length;
    float angle;
    Eigen::Vector3f org;
    Eigen::Vector3f dir;
    Cone (float _side_length, float _angle, Eigen::Vector3f _org, Eigen::Vector3f _dir):
        side_length(_side_length), angle(_angle), org(_org), dir (_dir)
    {}

    bool isInside(Eigen::Vector3f point)
    {
        return acos(dir.dot((point-org).normalized())) <= angle/2.0 && (point-org).norm() < side_length;
    }

    Cone(Eigen::Vector3f _org, Eigen::Vector3f _left_pt, Eigen::Vector3f _right_pt, float _side_length)
    {
        org = _org;
        side_length = _side_length; // ((org  - (_left_pt+_right_pt)/2.0).norm() + _side_length)/2.0;
        Eigen::Vector3f left_vec = (_left_pt-org).normalized();
        Eigen::Vector3f right_vec = (_right_pt-org).normalized();
        angle = acos(left_vec.dot(right_vec));
        dir = (left_vec + right_vec).normalized();
    }
};


void computeCombinations(std::vector< std::vector<int> > &array,
                         std::vector< std::vector<int> > &comb,
                         int i, std::vector<int> accum);

void computePermutations(int size, std::vector< std::vector<int> > &permutations);

void subsampleInt(Terrain *terrain, std::vector<Eigen::Vector3f> &points, int eps = 1);
void subsample(std::vector<Eigen::Vector3f> &points, float eps = 1);

FeatureMatching::FeatureMatching(Terrain *terrain, std::vector<Sketch*> &sketches,
                                 std::vector< std::vector<Terrain::SilhouetteSegment> > &features):
    terrain_(terrain), sketches_(sketches), features_(features)
{
}

void FeatureMatching::update(DeformationWeighting weighting)
{
    weighting_ = weighting;

    for (unsigned int i = 0; i < features_.size(); ++i)
    {
        for (unsigned int j = 0; j < features_.size(); ++j)
        {
            Terrain::SilhouetteSegment& seg = features_[i][j];
            subsampleInt(terrain_, seg.points, 1);
        }
    }

    computeProjections();

    std::cout << "Compute Assignments with number of features: "
              << projected_sketches_.size() << std::endl << std::flush;
    computeAssignments();

    computeDeformedFeatures(); //deform();

    std::cout << "Done computing assignments: " << deformed_features_.size() << "\n\n" << std::flush;

    deformed_features_per_sketch.clear();
}

void FeatureMatching::computeAssignments()
{
    bool *available = new bool [terrain_->width()*terrain_->height()];
    for (int j = 0; j < terrain_->height(); ++j)
        for (int i = 0; i < terrain_->width(); ++i)
            available[j*terrain_->width() + i] = true;

    deformed_features_per_sketch.clear();
    std::vector<Cone> unavailable_vis_cones;

    float terrain_diagonal_length = sqrt((float)(terrain_->width()*terrain_->width() + terrain_->height()*terrain_->height()));

    deformed_features_per_sketch.clear();
    deformed_features_per_sketch.resize(sketches_.size());

    for (int i = 0; i < sketches_.size(); ++i)
     {
        Sketch *sketch_ = sketches_[i];
        Eigen::Vector3f eye = sketch_->camera_info().position;

        for (int m = 0; m < sketch_->numStrokes(); ++m)
        {
            std::vector<PolylineCut> cuts;
            computePossibleCutsForStroke(i, m, cuts);

            Stroke *stroke = sketch_->getStroke(m);

            std::vector<PolylineCut> selected;
            int stroke_left_pos = 0;

            /*if (m > 2){
                deformed_features_per_sketch.push_back(selected);
                continue;
            }*/

            while (stroke_left_pos < sketch_->getStroke(m)->numControlPoints())
            {
                for (unsigned int i = 0; i < cuts.size(); ++i)
                {
                    cuts[i].cost = 0;

                    //TODOD Crosscheck this condition for unavailable features
                    int sx = round(cuts[i].deformed_points.front()[0]), sy = round(cuts[i].deformed_points.front()[1]);
                    int ex = round(cuts[i].deformed_points.back()[0]), ey = round(cuts[i].deformed_points.back()[1]);
                    if (!available[ey*terrain_->width() + ex] || !available[sy*terrain_->width() + sx])
                        cuts[i].cost = FLT_MAX/2;

                    for (unsigned int k=0; k < unavailable_vis_cones.size(); ++k)
                        if (unavailable_vis_cones[k].isInside(cuts[i].deformed_points.front())
                                || unavailable_vis_cones[k].isInside(cuts[i].deformed_points.back()))
                        {
                            cuts[i].cost = FLT_MAX/2;
                            break;
                        }

                    if (stroke_left_pos >= cuts[i].stroke_interval.second)
                        cuts[i].cost = FLT_MAX/2;

                    float deformation_cost = 0.0;
                    for (unsigned int j=0; j< cuts[i].deformed_points.size(); ++j)
                    {
                        Eigen::Vector3f point  = cuts[i].deformed_points[j];
                        deformation_cost += pow((point[2] - terrain_->getAltitude(((int)point[0]), ((int)point[1]))), 2);
                    }
                    deformation_cost /= cuts[i].deformed_points.size();

                    /*cuts[i].cost += weighting_.weights[kLeftMostFeatureWeight]*std::abs(cuts[i].stroke_interval.first - stroke_left_pos)/(sketch_->getStroke(m)->numControlPoints());
                    cuts[i].cost += weighting_.weights[kRightMostFeatureWeight]*std::abs(cuts[i].stroke_interval.second - sketch_->getStroke(m)->numControlPoints())/(sketch_->getStroke(m)->numControlPoints());
                    cuts[i].cost += weighting_.weights[kLongestFeatureWeight]*1.0/((FLT_EPSILON + std::abs(cuts[i].stroke_interval.second - cuts[i].stroke_interval.first))/sketch_->getStroke(m)->numControlPoints());
                    cuts[i].cost += weighting_.weights[kFrontMostFeatureWeight]*(sketch_->camera_info().position - cuts[i].deformed_points.front()).norm()/terrain_diagonal_length;

                    cuts[i].cost += weighting_.weights[kLeastDeformedFeatureWeight]*deformation_cost;
                    */
                    }
                std::sort(cuts.rbegin(), cuts.rend(), lessPolylineCut);

                PolylineCut cut = cuts.back();
                if (stroke_left_pos >= cut.stroke_interval.second)
                {
                    break;
                }

                selected.push_back(cut); cuts.pop_back();
                stroke_left_pos = cut.stroke_interval.second;

                fprintf(stdout, "Added to stroke %d, step %d: %d-%d out of %d-%d\n", m, selected.size(),
                        cut.stroke_interval.first, cut.stroke_interval.second, 0, stroke->numControlPoints());
                fflush(stdout);
            }

            fprintf(stdout, "\n");

            for (unsigned int i=0; i< selected.size(); ++i)
            {
                for (unsigned int j=0; j< selected[i].deformed_points.size(); ++j)
                {
                    Eigen::Vector3f point  = selected[i].deformed_points[j];
                    int x = round(point[0]), y = round(point[1]);
                    available[y*terrain_->width() + x] = false;
                }

                Stroke *stroke = sketch_->getStroke(m);
                Eigen::Vector3f left_pt = stroke->getControlPoint(0);
                Eigen::Vector3f right_pt = stroke->getControlPoint(stroke->numControlPoints()-1);
                Cone cone(eye, left_pt, right_pt, selected[i].furthest_dist_to_eye);
                unavailable_vis_cones.push_back(cone);
            }
            deformed_features_per_sketch[i].push_back(selected);
        }
      }

    delete [] available;
}

void FeatureMatching::computeDeformedFeatures()
{
  for (unsigned int sketch_id=0; sketch_id<deformed_features_per_sketch.size(); ++sketch_id)
  {
    for (int m=0; m < deformed_features_per_sketch[sketch_id].size(); ++m)
    {
        for (unsigned int i=0; i<deformed_features_per_sketch[sketch_id][m].size(); ++i)
        {
            Terrain::SilhouetteSegment seg;
            seg.points = deformed_features_per_sketch[sketch_id][m][i].deformed_points;
            deformed_features_.push_back(seg);
        }
    }
  }
}

void FeatureMatching::computePossibleCutsForStroke(int sketch_id, int stroke_id, std::vector<PolylineCut> &polyline_cuts)
{
    polyline_cuts.clear();
    const Terrain::SilhouetteSegment& stroke_seg = projected_sketches_[sketch_id][stroke_id];

    for (unsigned int i = 0; i < projected_features_[sketch_id].size(); ++i)
    {
        const Terrain::SilhouetteSegment& seg = projected_features_[sketch_id][i];
        PolylineCut cut(i); cut.sketch_id = sketch_id;
        int first_stroke_pos = -1;
        int prev_stroke_pos = -1;

        for (unsigned int j = 0; j < seg.points.size(); ++j)
        {
            float t,s; bool does_intersect = false;
            int stroke_pos = -1;
            for (int k = 0; k < ((int)stroke_seg.points.size())-1;++k)
            {
                Eigen::Vector3f sketch_point1 = stroke_seg.points[k];
                Eigen::Vector3f sketch_point2 = stroke_seg.points[k+1];
                if (intersectLineWithSegment(seg.points[j], Eigen::Vector3f(0,0,1),
                                             sketch_point1, sketch_point2,
                                             t, s))
                {
                    does_intersect = true;
                    stroke_pos = k;
                    break;
                }
            }
            if (does_intersect)
            {
                if (cut.left_pos < 0)
                {
                    cut.left_pos = j;
                    cut.stroke_interval.first = stroke_pos;
                    first_stroke_pos = stroke_pos;
                }

                Eigen::Vector3f point = seg.points[j] + t*Eigen::Vector3f(0,0,1);
                float real_dist = (features_[sketch_id][i].points[j] - sketches_[sketch_id]->camera_info().position).norm();
                float prj_dist = (projected_features_[sketch_id][i].points[j] - sketches_[sketch_id]->camera_info().position).norm();
                float deformed_height = t*(real_dist)/prj_dist;
                point = features_[sketch_id][i].points[j]; point[2] += deformed_height;

                if (cut.left_pos != ((int)j) && prev_stroke_pos != first_stroke_pos && stroke_pos != prev_stroke_pos)
                {
                    //TODO Fix this condition. Probably buggy (it checks if the feature is in the opposite dir)
                    if ((stroke_pos <= prev_stroke_pos) ^ (prev_stroke_pos < first_stroke_pos))
                    {
                        if (cut.left_pos != cut.right_pos) polyline_cuts.push_back(cut);
                        cut.left_pos = j;
                        cut.stroke_interval.first = stroke_pos;
                        cut.right_pos = -1;
                        cut.deformed_points.clear();
                        first_stroke_pos = stroke_pos;
                    }
                }

                if (cut.deformed_points.size() > 0)
                    cut.longest_edge_length = std::max(cut.longest_edge_length,
                                                       (point-cut.deformed_points.back()).norm());
                else
                    cut.longest_edge_length = 0;

                cut.deformed_points.push_back(point);
                cut.right_pos = j;
                cut.stroke_interval.second = stroke_pos;
                prev_stroke_pos = stroke_pos;
            } else
            {
                if (cut.left_pos >= 0)
                {
                    if (cut.left_pos != cut.right_pos) polyline_cuts.push_back(cut);
                    cut.left_pos = cut.right_pos = -1;
                    cut.deformed_points.clear();
                }
            }
        }
        if (cut.left_pos >= 0)  polyline_cuts.push_back(cut);
    }

    for (unsigned int i = 0; i < polyline_cuts.size(); ++i)
    {
        if (polyline_cuts[i].stroke_interval.second < polyline_cuts[i].stroke_interval.first)
        {
            int tmp = polyline_cuts[i].stroke_interval.second;
            polyline_cuts[i].stroke_interval.second = polyline_cuts[i].stroke_interval.first;
            polyline_cuts[i].stroke_interval.first = tmp;

            tmp = polyline_cuts[i].left_pos;
            polyline_cuts[i].left_pos = polyline_cuts[i].right_pos;
            polyline_cuts[i].right_pos = tmp;

            std::reverse(polyline_cuts[i].deformed_points.begin(), polyline_cuts[i].deformed_points.end());
        }

        polyline_cuts[i].furthest_dist_to_eye = -FLT_MAX;
        polyline_cuts[i].closest_dist_to_eye = FLT_MAX;
        for (unsigned int j=0; j< polyline_cuts[i].deformed_points.size(); ++j)
        {
            Eigen::Vector3f point  = polyline_cuts[i].deformed_points[j];
            float dist_to_eye = (sketches_[sketch_id]->camera_info().position - point).norm();
            polyline_cuts[i].furthest_dist_to_eye = std::max(polyline_cuts[i].furthest_dist_to_eye, dist_to_eye);
            polyline_cuts[i].closest_dist_to_eye = std::min(polyline_cuts[i].closest_dist_to_eye, dist_to_eye);
        }
    }
}

void FeatureMatching::splitOndulatingFeatures()
{
    std::vector< std::vector<Terrain::SilhouetteSegment> > split_features(sketches_.size());
    for (unsigned int sketch_id  = 0; sketch_id < sketches_.size(); ++sketch_id)
    {
        for (unsigned int i = 0; i < features_.size(); ++i)
        {
            Terrain::SilhouetteSegment& seg = features_[sketch_id][i];
            Terrain::SilhouetteSegment cur;

            int npoints = (int)seg.points.size();
            for (int j = 0; j < npoints; ++j)
            {
                if (j==0 || j==npoints-1)
                {
                    cur.points.push_back(seg.points[j]);
                } else
                {
                    Eigen::Vector3f vec1 = (seg.points[j] - seg.points[j-1]).normalized();
                    Eigen::Vector3f vec2 = (seg.points[j+1] - seg.points[j]).normalized();
                    if (cur.points.size() > 2 && acos(vec1.dot(vec2)) > M_PI_4)
                    {
                        split_features[sketch_id].push_back(cur);
                        cur.points.clear();
                    }
                    cur.points.push_back(seg.points[j]);
                }
            }
            if (cur.points.size() > 1)   split_features[sketch_id].push_back(cur);
        }
    }
    features_ = split_features;
}

void FeatureMatching::computeProjections()
{
    projected_features_.clear(); projected_features_.resize(sketches_.size());
    projected_sketches_.clear(); projected_sketches_.resize(sketches_.size());

    for (unsigned int sketch_id  = 0; sketch_id < sketches_.size(); ++sketch_id)
    {
      Sketch *sketch = sketches_[sketch_id];

      for (unsigned int i = 0; i < features_[sketch_id].size(); ++i)
      {
          Terrain::SilhouetteSegment& seg = features_[sketch_id][i];
          Terrain::SilhouetteSegment projected_seg;
          for (unsigned int j = 0; j < seg.points.size(); ++j)
              projected_seg.points.push_back(projectOnSketchPlane(sketch_id, seg.points[j]));
          projected_features_[sketch_id].push_back(projected_seg);
      }

      for (int m=0; m < sketch->numStrokes(); ++m)
      {
          const Stroke *stroke = sketch->getStroke(m);
          Terrain::SilhouetteSegment projected_seg;
          for (int j = 0; j < stroke->numControlPoints(); ++j)
              projected_seg.points.push_back(projectOnSketchPlane(sketch_id, stroke->getControlPoint(j)));
          projected_sketches_[sketch_id].push_back(projected_seg);
      }
    }
}

Eigen::Vector3f FeatureMatching::projectOnSketchPlane(int sketch_id, Eigen::Vector3f point)
{
    Sketch *sketch = sketches_[sketch_id];

    Eigen::Vector3f point_on_sketch = sketch->getStroke(0)->getControlPoint(0);

    Eigen::Vector3f ray_dir = (point - sketch->camera_info().position).normalized();
    return  projectOnPlane(point_on_sketch, -sketch->camera_info().direction,
                           sketch->camera_info().position, ray_dir);
}

void computePermutations(int size, std::vector< std::vector<int> > &permutations)
{
    std::vector<int> list(size);
    for (int i = 0; i < size; ++i)
        list[i] = i;

    do {
        permutations.push_back(list);
        //for (int i = 0; i < size; ++i) fprintf(stderr, "%d ", list[i]);
        //fprintf(stderr, "\n"); fflush(stderr);
    } while ( std::next_permutation(list.begin(),list.end()) );
}

void computeCombinations(std::vector< std::vector<int> > &array,
                         std::vector< std::vector<int> > &comb,
                         int i, std::vector<int> accum)
{
    if (i == ((int)array.size())) // done, no more rows
    {
        comb.push_back(accum); // assuming comb is global
        for (unsigned int j=0; j<accum.size(); ++j)
            fprintf(stderr, "%d ", accum[j]);
        fprintf(stderr, "\n"); fflush(stderr);
    }
    else
    {
        std::vector<int> row = array[i];
        for(unsigned int j = 0; j < row.size(); ++j)
        {
            std::vector<int> tmp(accum);
            tmp.push_back(row[j]);
            computeCombinations(array,comb,i+1,tmp);
        }
    }
}

void subsampleInt(Terrain *terrain, std::vector<Eigen::Vector3f> &points, int eps)
{
    std::vector<Eigen::Vector3f> new_points;

    for (int i = 0; i < ((int)points.size())-1; ++i)
    {
        Eigen::Vector3f point0 = points[i];
        Eigen::Vector3f point1 = points[i+1];

        int x0 = round(point0[0]);
        int y0 = round(point0[1]);
        int x1 = round(point1[0]);
        int y1 = round(point1[1]);

        float deltax = x1 - x0;
        float deltay = y1 - y0;

        int ystep;
        if (y0 < y1) ystep = 1;
        else ystep = -1;

        int xstep;
        if (x0 < x1) xstep = 1;
        else xstep = -1;

        /*
        float length = sqrt(deltax*deltax + deltay*deltay);
        int n  = length/eps;
        float dirx = deltax/length;
        float diry = deltay/length;

        for (int i = 0; i < n; ++i)
        {
            float t = ((float)i)/n;
            int x = ((float)x0 + ((float)t*dirx));
            int y = ((float)y0 + ((float)t*diry));
            Eigen::Vector3f point(x, y, terrain->getAltitude(x, y));

            if (new_points.size()>0 && (point-new_points.back()).norm() < FLT_EPSILON)
                continue;
            else
                new_points.push_back(point);
        }
        new_points.push_back(point1);
        */

        if (deltax < FLT_EPSILON)
        {
            int y = y0;
            while (y < y1)
            {
                Eigen::Vector3f point(x0, y, terrain->getAltitude(x0, y));
                new_points.push_back(point);
                y += ystep*eps;
            }
        } else {
            float error = 0;
            float deltaerr = std::abs (deltay / deltax);
            int y = y0;
            for (int x = x0; (xstep>0)?x<x1:x>x1; x+=xstep*eps)
            {
                Eigen::Vector3f point(x, y, terrain->getAltitude(x, y));
                new_points.push_back(point);

                error = error + eps*deltaerr;
                if (error >= 0.5)
                {
                    y += ystep*eps;
                    error = error - 1.0;
                }
                //new_points.push_back(point0);
            }
        }
        new_points.push_back(point1);
    }

    points = new_points;
}
