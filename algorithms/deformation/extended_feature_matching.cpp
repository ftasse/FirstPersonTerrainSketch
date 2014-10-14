#include "extended_feature_matching.h"
#include "tools/silhouette_extractor.h"

#include <float.h>
#include <stdlib.h>
#include <Eigen/Geometry>

#define kAlpha 0.01

ExtendedFeatureMatching::ExtendedFeatureMatching(Terrain *terrain, std::vector< Sketch *> &sketches,
                                                 std::vector< std::vector<Terrain::SilhouetteSegment> > &silhouettes):
    FeatureMatching(terrain, sketches, silhouettes)
{
}

float ExtendedFeatureMatching::branch(std::vector< std::vector< std::vector<PolylineCut> > > &cuts_per_stroke,
                                         std::vector<Polygon> &polygon_regions,
                                         std::vector< std::vector< std::vector<PolylineCut> > > &cur_deformed_features_per_sketch,
                                         float &cur_best_cost,
                                         float cur_cost,
                                         int cur_sketch_id,
                                         int cur_stroke_id,
                                         int cur_cut_id)
{
    //fprintf(stdout, "Process: %d %d %d\n", cur_stroke_id, cur_cut_id, polygon_regions.size());
    //fflush(stdout);

    std::vector<PolylineCut> cuts = cuts_per_stroke[cur_sketch_id][cur_stroke_id];

    if (cur_cut_id < 0) cur_cost += 1e8;
    else cur_cost += cuts[cur_cut_id].cost;

    if (cur_cost >= cur_best_cost - FLT_EPSILON) return cur_cost;

    std::vector< std::vector< std::vector<PolylineCut> > > tmp = cur_deformed_features_per_sketch;
    if (cur_cut_id < 0) tmp[cur_sketch_id].push_back(std::vector<PolylineCut>());
    else tmp[cur_sketch_id].push_back(std::vector<PolylineCut>(1, cuts[cur_cut_id]));

    int best_cut_id = -1;

    if (cur_stroke_id < sketches_[cur_sketch_id]->numStrokes()-1)
    {
        float best_cost = 1e8;

        std::vector<Polygon> new_poly_regions = polygon_regions;
        Polygon polygon;
        if (cur_cut_id >= 0)
        {
          buildPolygonFrom(cuts[cur_cut_id], true, polygon);
          new_poly_regions.push_back(polygon);
        }

        bool valid = false;
        for (unsigned int i = 0; i < cuts_per_stroke[cur_sketch_id][cur_stroke_id+1].size(); ++i)
            if (isCutAvailable(cuts_per_stroke[cur_sketch_id][cur_stroke_id+1][i], new_poly_regions))
            {
                valid = true;
                float cost = branch(cuts_per_stroke, new_poly_regions, tmp,
                                    cur_best_cost, cur_cost, cur_sketch_id, cur_stroke_id+1, i);
                if (cost < best_cost)
                {
                    best_cost = cost;
                    best_cut_id = i;

                    if (cur_stroke_id+1 == sketches_[cur_sketch_id]->numStrokes()-1)
                        break;
                }

            }

        if (best_cut_id < 0)
        {
            best_cost = branch(cuts_per_stroke, new_poly_regions, tmp,
                                cur_best_cost, cur_cost, cur_sketch_id, cur_stroke_id+1, -1);
        }

        cur_cost = best_cost;
    }

    if (cur_sketch_id < (((int) sketches_.size()) - 1) && cur_stroke_id == sketches_[cur_sketch_id]->numStrokes()-1)
    {
        std::vector<Polygon> new_poly_regions = polygon_regions;
        Polygon polygon;
        for (int i = 0; i <= cur_sketch_id; ++i)
          for (int j = 0; j < tmp[i].size(); ++j)
            for (int k = 0; k < tmp[i][j].size(); ++k)
              {
                buildPolygonFrom(tmp[i][j][k], true, polygon);
                new_poly_regions.push_back(polygon);
              }

        float best_cost = cur_best_cost;

        std::vector< std::vector< std::vector<PolylineCut> > > tmp2 = tmp;

        for (unsigned int i = 0; i < cuts_per_stroke[cur_sketch_id+1][0].size(); ++i)
        {
            if (isCutAvailable(cuts_per_stroke[cur_sketch_id+1][0][i],new_poly_regions))
            {
                std::vector< std::vector< std::vector<PolylineCut> > > tmp3 = tmp;
                float cost = branch(cuts_per_stroke, new_poly_regions, tmp3,
                                    cur_best_cost, cur_cost, cur_sketch_id+1, 0, i);
                if (cost < best_cost)
                {
                    best_cost = cost;
                    tmp2 = tmp3;
                }
            }
        }

        tmp = tmp2;

        cur_cost = best_cost;
    }


    if (cur_sketch_id == (((int) sketches_.size()) - 1) && cur_stroke_id == sketches_[cur_sketch_id]->numStrokes()-1 && cur_cost < cur_best_cost)
    {
        cur_best_cost = (1-kAlpha)*cur_cost;

        deformed_features_per_sketch = tmp;
        fprintf(stdout, "Selected: (%d, %d) -- %d %f\n",cur_sketch_id, cur_stroke_id, cur_cut_id, cur_best_cost);

        fflush(stdout);

//        static bool test = false;
//        if (!test) {cur_best_cost = 0.0;
//        test = true;}

    }
    return cur_cost;
}

void ExtendedFeatureMatching::computeAssignments()
{
    //Assign a set of silh-segments per stroke (deformed_features_per_sketch[i])

    std::vector< std::vector< std::vector<PolylineCut> > > cuts_per_stroke(sketches_.size());

    for (int i = 0; i < sketches_.size(); ++i)
    {
      cuts_per_stroke[i].resize(sketches_[i]->numStrokes());
      std::cout << "Process sketch " << i << "\n";

      for (int m = 0; m < sketches_[i]->numStrokes(); ++m)
      {
          std::vector<PolylineCut> cuts;
          computePossibleCutsForStroke(i, m, cuts);
          computeCutsCosts(i, m, cuts);
          std::sort(cuts.begin(), cuts.end(), lessPolylineCut);
          cuts_per_stroke[i][m] = cuts;

          //deformed_features_per_sketch.push_back(cuts_per_stroke[m]);
          std::cout << "Cuts Size for " << m << ": " <<  cuts_per_stroke[i][m].size() << std::endl << std::flush;
      }

      std::cout << "\n";
    }

    std::vector<Polygon> poly_regions;
    std::vector< std::vector< std::vector<PolylineCut> > > cur_deformed_features_per_sketch(sketches_.size());
    float cur_best_cost = FLT_MAX;

    clock_t begin = clock();

    for (unsigned int i = 0; i < cuts_per_stroke[0][0].size(); ++i)
      if (isCutAvailable(cuts_per_stroke[0][0][i], poly_regions))
        branch(cuts_per_stroke, poly_regions, cur_deformed_features_per_sketch, cur_best_cost, 0, 0, 0, i);

    if (deformed_features_per_sketch.size() == 0)
      deformed_features_per_sketch.resize(sketches_.size());

    for (unsigned int sketch_id = 0; sketch_id < sketches_.size(); ++sketch_id)
    {
        Sketch * sketch_ = sketches_[sketch_id];

        if (deformed_features_per_sketch[sketch_id].size()!=sketch_->numStrokes())
          deformed_features_per_sketch[sketch_id].resize(sketch_->numStrokes());

        for (int m = 0; m < sketch_->numStrokes(); ++m)
        {
            if (deformed_features_per_sketch[sketch_id][m].size() == 0)
              {
                std::cout << "Add a deformation constraint for unassigne stroke " << m << std::endl << std::flush;
                Stroke *stroke = sketch_->getStroke(m);
                Eigen::Vector3f eye = sketch_->camera_info().position;

                if (m == 0)
                  {
                    PolylineCut cut;
                    for (int i = 0; i<stroke->numControlPoints(); ++i)
                      cut.deformed_points.push_back(stroke->getControlPoint(i));
                    deformed_features_per_sketch[sketch_id][m] = std::vector<PolylineCut>(1, cut);
                  }

                else
                  {

                    PolylineCut front, back;

                    Eigen::Vector3f plane_point;
                    float min_dist_eye = -FLT_MAX;
                    front = deformed_features_per_sketch[sketch_id][m-1][0];

                    for (int i = 0; i < front.deformed_points.size(); ++i)
                      {
                        min_dist_eye = std::max(min_dist_eye, (eye - front.deformed_points[i]).norm());
                      }

                      if (m + 1 < sketch_->numStrokes() && deformed_features_per_sketch[m+1].size() > 0)
                      {
                        back = deformed_features_per_sketch[sketch_id][m+1][0];
                        float max_dist_eye = FLT_MAX;

                        for (int i = 0; i < back.deformed_points.size(); ++i)
                          {
                            max_dist_eye = std::min(max_dist_eye, (eye - back.deformed_points[i]).norm());
                          }

                        plane_point = eye + ((max_dist_eye - min_dist_eye)/2.0)*sketch_->camera_info().direction.normalized();

                      } else
                      {
                        plane_point = eye + (min_dist_eye+30)*sketch_->camera_info().direction.normalized();
                      }

                    //plane_point = stroke->getControlPoint(0);

                    PolylineCut cut;
                    for (int i = 0; i<stroke->numControlPoints(); ++i)
                      {
                        Eigen::Vector3f point;
                        point = projectOnPlane(plane_point, -sketch_->camera_info().direction.normalized(), eye, (stroke->getControlPoint(i) - eye).normalized());
                        point[0] = round(point[0]);
                        point[1] = round(point[1]);
                        if (terrain_->validCoordinates(point[0], point[1])) cut.deformed_points.push_back(point);
                      }
                    deformed_features_per_sketch[sketch_id][m] = std::vector<PolylineCut>(1, cut);
                  }
              }
        }

      }

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Branch and bound: " << elapsed_secs << " secs" << std::endl << std::flush;
}


void ExtendedFeatureMatching::computePossibleCutsForStroke(int sketch_id, int stroke_id,
                                                           std::vector<PolylineCut> &polyline_cuts)
{
    Sketch *sketch_ = sketches_[sketch_id];

    std::vector<PolylineCut> cuts;
    FeatureMatching::computePossibleCutsForStroke(sketch_id, stroke_id, cuts);

    Stroke *stroke = sketch_->getStroke(stroke_id);
    Eigen::Vector3f eye = sketch_->camera_info().position;

    for (unsigned int i = 0; i < cuts.size(); ++i)
    {
        if (std::abs(cuts[i].stroke_interval.second - cuts[i].stroke_interval.first) < stroke->numControlPoints()/4)
          continue;

        bool invalid = false;
        cuts[i].nb_extruded = cuts[i].stroke_interval.first +
                (stroke->numControlPoints()-1 - cuts[i].stroke_interval.second);

        if (cuts[i].stroke_interval.first > cuts[i].stroke_interval.second)
        {
            std::reverse(cuts[i].deformed_points.begin(), cuts[i].deformed_points.end());
            int tmp  =cuts[i].stroke_interval.first;
            cuts[i].stroke_interval.first = cuts[i].stroke_interval.second;
            cuts[i].stroke_interval.second = tmp;
        }

        if (cuts[i].stroke_interval.first > 0) {
            for (int k = cuts[i].stroke_interval.first-1; k >= 0; k--)
            {
                Eigen::Vector3f plane_vec1 = (stroke->getControlPoint(k+1) - stroke->getControlPoint(k)).normalized(); //(cuts[i].deformed_points[1] - cuts[i].deformed_points[0]).normalized();
                Eigen::Vector3f plane_vec2 = Eigen::Vector3f(0, 0, 1);

                if ((plane_vec1-plane_vec2).norm() < 1e-5)
                    plane_vec1 = (cuts[i].deformed_points[1] - cuts[i].deformed_points[0]).normalized();
                Eigen::Vector3f plane_normal = plane_vec1.cross(plane_vec2).normalized();

                Eigen::Vector3f left_most_point = projectOnPlane(cuts[i].deformed_points[0], plane_normal,
                                                                 eye, (stroke->getControlPoint(k)-eye).normalized());

                if (isnan(left_most_point[0]))
                {
                    invalid = true;
                    break;
                }

                left_most_point[0]  =round(left_most_point[0]);
                left_most_point[1]  =round(left_most_point[1]);

                if (!terrain_->validCoordinates(left_most_point[0], left_most_point[1]))
                {
                    invalid = true;
                    break;
                }

                cuts[i].deformed_points.insert(cuts[i].deformed_points.begin(), left_most_point);
                cuts[i].stroke_interval.first--;
            }
        }

        if (cuts[i].stroke_interval.second < stroke->numControlPoints()-1) {
            for (int k = cuts[i].stroke_interval.second+1; k < stroke->numControlPoints(); k++)
            {
                Eigen::Vector3f plane_vec1 = (stroke->getControlPoint(k) - stroke->getControlPoint(k-1)).normalized(); //(cuts[i].deformed_points[1] - cuts[i].deformed_points[0]).normalized();
                Eigen::Vector3f plane_vec2 = Eigen::Vector3f(0, 0, 1);

                if ((plane_vec1-plane_vec2).norm() < 1e-5)
                    plane_vec1 = (cuts[i].deformed_points.back() -
                                  cuts[i].deformed_points[cuts[i].deformed_points.size()-1]).normalized();

                Eigen::Vector3f plane_normal = plane_vec1.cross(plane_vec2).normalized();

                Eigen::Vector3f right_most_point = projectOnPlane(cuts[i].deformed_points.back(), plane_normal,
                                                                  eye, (stroke->getControlPoint(k)-eye).normalized());

                if (isnan(right_most_point[0]))
                {
                    invalid = true;
                    break;
                }

                right_most_point[0]  =round(right_most_point[0]);
                right_most_point[1]  =round(right_most_point[1]);

                if (!terrain_->validCoordinates(right_most_point[0], right_most_point[1]))
                {
                    invalid = true;
                    break;
                }

                cuts[i].deformed_points.insert(cuts[i].deformed_points.end(), right_most_point);
                cuts[i].stroke_interval.second++;
            }
        }

        if (!invalid)
            polyline_cuts.push_back(cuts[i]);
    }
}

void ExtendedFeatureMatching::computeCutsCosts(int sketch_id, int stroke_id,
                                               std::vector<PolylineCut> &polyline_cuts)
{
    Sketch *sketch_ = sketches_[sketch_id];

    float terrain_diagonal_length = sqrt((float)(terrain_->width()*terrain_->width() + terrain_->height()*terrain_->height()));
    Eigen::Vector3f eye = sketch_->camera_info().position;

    float sum_longest_edge_lengths = 0;
    float max_longest_edge_length = -FLT_MAX;
    for (unsigned int i = 0; i < polyline_cuts.size(); ++i)
    {
        sum_longest_edge_lengths += polyline_cuts[i].longest_edge_length;
        max_longest_edge_length = std::max(max_longest_edge_length, polyline_cuts[i].longest_edge_length);
    }

    for (unsigned int i = 0; i < polyline_cuts.size(); ++i)
    {
        polyline_cuts[i].cost = 0.0;

        float similarity_cost = 0.0;
        float deformation_cost = 0.0;
        float max_similarity_cost = 0.0;
        float max_deformation_cost = 0.0;

        for (unsigned int k=0; k<polyline_cuts[i].deformed_points.size()-1; ++k)
        {
            Eigen::Vector3f deformed = polyline_cuts[i].deformed_points[k];
            Eigen::Vector3f onterrain = deformed; onterrain[2] = terrain_->getAltitude(deformed[0], deformed[1]);
            deformation_cost += std::abs(deformed[2] - onterrain[2]) *
                    sqrt(pow(deformed[1] - onterrain[1], 2) + pow(deformed[0] - onterrain[0], 2));
            //max_deformation_cost = std::max(std::abs(deformed[2] - onterrain[2]), max_deformation_cost);

            Eigen::Vector3f deformed_prj = projectOnSketchPlane(sketch_id, deformed);
            Eigen::Vector3f onterrain_prj = projectOnSketchPlane(sketch_id, onterrain);
            similarity_cost += std::abs(deformed_prj[2] - onterrain_prj[2]) *
                    sqrt(pow(deformed_prj[1] - onterrain_prj[1], 2) + pow(deformed_prj[0] - onterrain_prj[0], 2));
            //max_similarity_cost = std::max(std::abs(deformed_prj[2] - onterrain_prj[2]), max_similarity_cost);
        }

        similarity_cost /= polyline_cuts[i].deformed_points.size();
        deformation_cost /= polyline_cuts[i].deformed_points.size();

        float longest_edge_cost = polyline_cuts[i].longest_edge_length/max_longest_edge_length;
        float extended_feature_cost = ((float)polyline_cuts[i].nb_extruded)/polyline_cuts[i].deformed_points.size();

        polyline_cuts[i].cost += weighting_.weights[kSimilarityWeight]*similarity_cost;
        polyline_cuts[i].cost += weighting_.weights[kDeformationWeight]*deformation_cost;
        polyline_cuts[i].cost += weighting_.weights[kSamplingWeight]*longest_edge_cost;
        polyline_cuts[i].cost += weighting_.weights[kFeatureExtensionWeight]*extended_feature_cost;

        //fprintf(stdout, "Longest edge cost: %f\n", longest_edge_cost);
        //fflush(stdout);

        //polyline_cuts[i].cost += 2*(1/(polyline_cuts[i].deformed_points[0] - eye).norm())/(1/terrain_diagonal_length);
    }
}

bool ExtendedFeatureMatching::isCutAvailable(PolylineCut &cut) {
    return isCutAvailable(cut, unavailable_polygon_regions);
}

bool ExtendedFeatureMatching::isCutAvailable(PolylineCut &cut, std::vector<Polygon> &regions) {

    // Check if feature cut is in front of a previously assigned feature
    for (unsigned int p = 0; p < regions.size(); ++p)
        if (regions[p].isInside(cut))
            return false;

    Polygon polygon;
    buildPolygonFrom(cut, true, polygon);

    // Check if feature cut contains any camera position
    for (unsigned int i = 0; i < sketches_.size(); ++i)
      if (cut.sketch_id != i)
      {
          if (polygon.isInside(sketches_[i]->camera_info().position))
            return false;
      }

    // Check if feature cut visibility intersects with any features assigned for another sketch
    for (unsigned int p = 0; p < regions.size(); ++p)
        if (regions[p].sketch_id != cut.sketch_id)
          {
            if (regions[p].intersects(polygon))
                return false;
          }

    return true;
}

void ExtendedFeatureMatching::buildPolygonFrom(PolylineCut &cut, bool front_area, Polygon &polygon)
{
    polygon.sketch_id = cut.sketch_id;

    Sketch *sketch_ = sketches_[cut.sketch_id];
    std::vector< std::pair<int,int> > polygon_points;
    float terrain_diagonal_length = sqrt((float)(terrain_->width()*terrain_->width() + terrain_->height()*terrain_->height()));
    Eigen::Vector3f eye = sketch_->camera_info().position;

    Eigen::Vector3f left_pt = cut.deformed_points.front();
    Eigen::Vector3f right_pt = cut.deformed_points.back();

    if (front_area)
    {
        left_pt += 20*(left_pt - eye).normalized();
        right_pt += 20*(right_pt - eye).normalized();
    } else
    {
        left_pt -= 20*(left_pt - eye).normalized();
        right_pt -= 20*(right_pt - eye).normalized();
    }

    if (front_area)
    {
        polygon_points.push_back(std::pair<int,int>(eye[0], eye[1]));
        polygon_points.push_back(std::pair<int,int>(left_pt[0], left_pt[1]));
        polygon_points.push_back(std::pair<int,int>(right_pt[0], right_pt[1]));
    } else
    {
        Eigen::Vector3f left_pt2 = left_pt + terrain_diagonal_length*(left_pt-eye).normalized();
        Eigen::Vector3f right_pt2 = right_pt + terrain_diagonal_length*(right_pt-eye).normalized();

        polygon_points.push_back(std::pair<int,int>(left_pt[0], left_pt[1]));
        polygon_points.push_back(std::pair<int,int>(left_pt2[0], left_pt2[1]));
        polygon_points.push_back(std::pair<int,int>(right_pt2[0], right_pt2[1]));
        polygon_points.push_back(std::pair<int,int>(right_pt[0], right_pt[1]));
    }

    polygon.points.clear();
    polygon.unavailable_positions.clear();

    for (unsigned int k = 0; k< polygon_points.size(); ++k)        {
        polygon.points.push_back(Eigen::Vector3f(polygon_points[k].first, polygon_points[k].second, 0));
    }


    for (unsigned int k=0; k<cut.deformed_points.size(); ++k)
    {
        int x = cut.deformed_points[k][0], y = cut.deformed_points[k][1];
        for (int m=-2; m<=2; ++m)
            for (int n=-2; n<=2; ++n)
                if (terrain_->validCoordinates(x+m, y+n))
                    polygon.unavailable_positions.insert(std::pair<int, int>(x+m, y+n));
    }
}

void ExtendedFeatureMatching::updateAvailabilityFrom(PolylineCut &cut, bool front_area)
{
    Polygon polygon;
    buildPolygonFrom(cut, front_area, polygon);
    unavailable_polygon_regions.push_back(polygon);
}
