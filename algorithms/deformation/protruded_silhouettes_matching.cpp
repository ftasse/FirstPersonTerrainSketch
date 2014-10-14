#include "protruded_silhouettes_matching.h"

#include "tools/silhouette_extractor.h"

ProtrudedSilhouetteMatching::ProtrudedSilhouetteMatching(Terrain *terrain, Sketch *sketch):
    terrain_(terrain), sketch_(sketch)
{
}

void ProtrudedSilhouetteMatching::updateSilhouettes()
{
    computeUnallocatedSilhouettes();
}

Eigen::Vector3f ProtrudedSilhouetteMatching::projectOnSketchPlane(Eigen::Vector3f point)
{
    Eigen::Vector3f point_on_sketch = sketch_->getStroke(0)->getControlPoint(0);

    Eigen::Vector3f ray_dir = (point - sketch_->camera_info().position).normalized();
    return  projectOnPlane(point_on_sketch, -sketch_->camera_info().direction,
                           sketch_->camera_info().position, ray_dir);
}


void ProtrudedSilhouetteMatching::computeUnallocatedSilhouettes()
{
    float max_sil_height = -FLT_MAX;
    Eigen::Vector3f eye = sketch_->camera_info().position;

    unallocated_silhouettes_.clear();
    double eps = 1e-2;

    bool *available = new bool [terrain_->width()*terrain_->height()];
    for (int j = 0; j < terrain_->height(); ++j)
        for (int i = 0; i < terrain_->width(); ++i)
            available[j*terrain_->width() + i] = true;

    for (unsigned int i = 0; i < deformed_features_.size(); ++i)
    {
        Terrain::SilhouetteSegment &seg = deformed_features_[i];

        for (unsigned int j=0; j < seg.points.size(); ++j)
        {
            Eigen::Vector3f sil_point = seg.points[j];
            int x = round(sil_point[0]);
            int y = round(sil_point[1]);

            if (seg.isSilhouette)
              for (int m=-5; m <=5; ++m)
                  for (int n=-5; n <= 5; ++n)
                  {
                      if (terrain_->validCoordinates(x+m, y+n))
                      {
                          available[(y+n)*terrain_->width() + x + m] = false;
                      }
                  }
            else
              for (int m=-10; m <=10; ++m)
                  for (int n=-10; n <= 10; ++n)
                  {
                      if (terrain_->validCoordinates(x+m, y+n))
                      {
                          available[(y+n)*terrain_->width() + x + m] = false;
                      }
                  }
        }
    }

    SilhouetteExtractor extractor(terrain_);
    std::vector<Terrain::SilhouetteSegment> silhouettes;
    extractor.compute(sketch_->camera_info(), silhouettes);

    for (unsigned int i = 0; i < silhouettes.size(); ++i)
    {
        Terrain::SilhouetteSegment &seg = silhouettes[i];
        std::vector<float> diff_alts(seg.points.size(), 0.0);
        float max_cur_sil_height = -FLT_MAX;

        for (unsigned int j=0; j < seg.points.size(); ++j)
        {
            Eigen::Vector3f sil_point = seg.points[j];
            Eigen::Vector3f prj_sil_point = projectOnSketchPlane(sil_point);

            int x = round(sil_point[0]);
            int y = round(sil_point[1]);
            if (available[y*terrain_->width() + x])
            {
                float min_t = -FLT_MAX;

                for (int m=0; m < sketch_->numStrokes(); ++m)
                {
                    Terrain::SilhouetteSegment assigned_seg = deformed_features_[m];
                    for (int k=0; k < ((int)assigned_seg.points.size())-1; ++k)
                    {
                       Eigen::Vector3f prj_point1 = projectOnSketchPlane(assigned_seg.points[k]);
                       Eigen::Vector3f prj_point2 = projectOnSketchPlane(assigned_seg.points[k+1]);

                        if ((Eigen::Vector2f(eye[0], eye[1]) - Eigen::Vector2f(sil_point[0], sil_point[1])).norm() >
                            (Eigen::Vector2f(eye[0], eye[1]) - Eigen::Vector2f(assigned_seg.points[k][0], assigned_seg.points[k][1])).norm())
                            continue;

                        float t, s;
                        if (intersectLineWithSegment(prj_sil_point, Eigen::Vector3f(0,0,1), prj_point1, prj_point2, t, s))
                        {
                            if (t <= 0 && t > min_t)
                            {
                                min_t = t;
                            }
                        }
                    }
                }

                float real_dist = (sil_point - sketch_->camera_info().position).norm();
                float prj_dist = (prj_sil_point - sketch_->camera_info().position).norm();
                if (min_t > -FLT_MAX + FLT_EPSILON && std::abs(min_t*(real_dist)/prj_dist) > 1)
                {
                    float diff = std::abs((min_t*(real_dist)/prj_dist))-eps;
                    max_sil_height = std::max(max_sil_height, diff);
                    max_cur_sil_height = std::max(max_cur_sil_height, diff);
                    diff_alts[j] = diff;
                } else
                {
                    diff_alts[j] = -FLT_MAX;
                }
            } else
            {
                diff_alts[j] = 0.0;
            }
        }

        if (max_cur_sil_height > -FLT_MAX + FLT_EPSILON)
        {
            //fprintf(stdout, "max sil displacement: %f\n", max_cur_sil_height);
            Terrain::SilhouetteSegment unallocated_seg;
            for (unsigned int j=0; j < seg.points.size(); ++j)
            {
                Eigen::Vector3f sil_point = seg.points[j];
                if (std::abs(diff_alts[j]) < FLT_EPSILON) // && ( j == seg.points.size()-1 || std::abs(diff_alts[j+1]) < FLT_EPSILON)
                        //&& ( j==0 || std::abs(diff_alts[j-1]) < FLT_EPSILON))
                {
                    if (unallocated_seg.points.size() > 1)  unallocated_silhouettes_.push_back(unallocated_seg);
                    unallocated_seg.points.clear();
                } else
                {
                    /*float diff = diff_alts[j];
                    //if (diff < -1e5)
                        diff = max_cur_sil_height;

                    sil_point[2] -= diff;*/
                    unallocated_seg.points.push_back(sil_point);
                }
            }
            unallocated_seg.isSilhouette = true;
            if (unallocated_seg.points.size() > 1)  unallocated_silhouettes_.push_back(unallocated_seg);
        }
    }

    delete [] available;

    for (unsigned int i = 0; i < unallocated_silhouettes_.size(); ++i)
        for (unsigned int j = 0; j < unallocated_silhouettes_[i].points.size(); ++j)
            unallocated_silhouettes_[i].points[j][2] -= max_sil_height;

    /*if (max_sil_height > -FLT_MAX + FLT_EPSILON )
    {
        //max_sil_height -= terrain_->min_altitude();
        fprintf(stdout, "Displacement: %f\n", max_sil_height);


        for (unsigned int x = 0;  x < terrain_->width(); ++x)
            for (unsigned int y = 0; y < terrain_->height(); ++y)
            {
                terrain_->setAltitude(x, y, terrain_->getAltitude(x, y) - max_sil_height);
            }
        terrain_->updateMinMax();
    }*/

    deformed_features_.insert(deformed_features_.end(),
                              unallocated_silhouettes_.begin(),
                              unallocated_silhouettes_.end());

    fprintf(stdout, "Num of unallocated silhouettes: %d\n", unallocated_silhouettes_.size());
    fflush(stdout);
}

/*void ProtrudedSilhouetteMatching::computeUnallocatedSilhouettes()
{
    unallocated_silhouettes_.clear();
    double eps= 1e-2;

    bool *available = new bool [terrain_->width()*terrain_->height()];
    for (int j = 0; j < terrain_->height(); ++j)
        for (int i = 0; i < terrain_->width(); ++i)
            available[j*terrain_->width() + i] = true;

    for (unsigned int i = 0; i < deformed_features_.size(); ++i)
    {
        Terrain::SilhouetteSegment &seg = deformed_features_[i];

        for (unsigned int j=0; j < seg.points.size(); ++j)
        {
            Eigen::Vector3f sil_point = seg.points[j];
            int x = round(sil_point[0]);
            int y = round(sil_point[1]);

            for (int m=-1; m <=1; ++m)
                for (int n=-1; n <= 1; ++n)
                {
                    if (terrain_->validCoordinates(x+m, y+n))
                    {
                        available[(y+n)*terrain_->width() + x + m] = false;
                    }
                }
        }
    }

    SilhouetteExtractor extractor(terrain_);
    std::vector<Terrain::SilhouetteSegment> silhouettes;
    extractor.compute(sketch_->camera_info(), silhouettes);

    for (unsigned int i = 0; i < silhouettes.size(); ++i)
    {
        Terrain::SilhouetteSegment &seg = silhouettes[i];
        Terrain::SilhouetteSegment unallocated_seg;

        for (unsigned int j=0; j < seg.points.size(); ++j)
        {
            bool close_to_stroke = false;
            Eigen::Vector3f sil_point = seg.points[j];
            Eigen::Vector3f prj_sil_point = projectOnSketchPlane(sil_point);

            int x = round(sil_point[0]);
            int y = round(sil_point[1]);
            if (available[y*terrain_->width() + x])
            {
                float min_t = -FLT_MAX;

                for (int m=0; m < sketch_->numStrokes(); ++m)
                {
                    if (close_to_stroke)    break;
                    Stroke *stroke = sketch_->getStroke(m);
                    for (int k=0; k < ((int)stroke->numControlPoints())-1; ++k)
                    {
                        Eigen::Vector3f prj_point1 = projectOnSketchPlane(stroke->getControlPoint(k));
                        Eigen::Vector3f prj_point2 = projectOnSketchPlane(stroke->getControlPoint(k+1));

                        float t, s;
                        if (intersectLineWithSegment(prj_sil_point, Eigen::Vector3f(0,0,1), prj_point1, prj_point2, t, s))
                        {
                            if (std::abs(t) < 1e-1)
                            {
                                close_to_stroke = true;
                                min_t = -FLT_MAX;
                                break;
                            }
                            if (t <= 0 && t > min_t)
                            {
                                min_t = t;
                            }
                        }
                    }
                }

                float real_dist = (sil_point - sketch_->camera_info().position).norm();
                float prj_dist = (prj_sil_point - sketch_->camera_info().position).norm();
                if (min_t > -FLT_MAX + FLT_EPSILON && std::abs(min_t*(real_dist)/prj_dist) > eps)
                {
                    sil_point[2] += (min_t*(real_dist)/prj_dist) - eps;
                    unallocated_seg.points.push_back(sil_point);
                } else
                {
                    if (unallocated_seg.points.size() > 1)  unallocated_silhouettes_.push_back(unallocated_seg);
                    unallocated_seg.points.clear();
                }
            } else
            {
                if (unallocated_seg.points.size() > 1)  unallocated_silhouettes_.push_back(unallocated_seg);
                unallocated_seg.points.clear();
                continue;
            }
        }

        if (unallocated_seg.points.size() > 1)  unallocated_silhouettes_.push_back(unallocated_seg);
        unallocated_seg.points.clear();
    }

    delete [] available;

    deformed_features_.insert(deformed_features_.end(),
                              unallocated_silhouettes_.begin(),
                              unallocated_silhouettes_.end());
}*/
