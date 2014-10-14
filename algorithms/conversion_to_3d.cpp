#include "conversion_to_3d.h"

#include <float.h>
#include <stdio.h>

#include <Eigen/Geometry>

struct StrokeInfo {
    int stroke_id;
    float max_alt;
    float min_alt;
    float xmin;
    float xmax;
    std::map<int, bool> junctions;  //yes if <this> is occluded
    StrokeInfo() {}

    bool operator <(const StrokeInfo &rhs) const
    {
        std::map<int, bool>::const_iterator mit = junctions.find(rhs.stroke_id);
        if (mit == junctions.end()) {
            if (xmin > rhs.xmin && xmax < rhs.xmax && min_alt > rhs.min_alt && max_alt < rhs.max_alt)
                return true;
            if (xmin < rhs.xmin && xmax > rhs.xmax && min_alt < rhs.min_alt && max_alt > rhs.max_alt)
                return false;
            return min_alt < rhs.min_alt;
        }
        else return !mit->second;
    }

};

bool lessXAxis(const StrokeInfo &lhs, const StrokeInfo &rhs)
{
   return lhs.xmin < rhs.xmin;
}

ConversionTo3D::ConversionTo3D(Sketch *sketch):
    sketch_(sketch)
{
}

ConversionTo3D::ConversionTo3D(const char *sketch_svg_path, Eigen::Vector2i terrain_size)
{
    sketch_ = new Sketch();
    sketch_->loadSvg(sketch_svg_path);
    sketch_->setDefaultCamera(terrain_size);
}

void ConversionTo3D::sortStrokes()
{
    //if (sorted_stroke_ids_.size() != sketch_->numStrokes())
        convert();

    sketch_->sortWithOrder(sorted_stroke_ids_);
}

#include <iostream>

void ConversionTo3D::convert()
{
    sorted_stroke_ids_.clear();

    if (sketch_->numStrokes() <= 1)
    {
        sorted_stroke_ids_.push_back(0);
        return;
    }
    
    //std::cout << "Mins \n " << std::flush;

    std::vector< std::vector<Eigen::Vector3f> > prj_stroke_pts(sketch_->numStrokes());
    std::vector<StrokeInfo> stroke_infos(sketch_->numStrokes());

    for (int i = 0; i < sketch_->numStrokes(); ++i)
    {
        stroke_infos[i].stroke_id = i;
        stroke_infos[i].max_alt = -FLT_MAX;
        stroke_infos[i].min_alt = FLT_MAX;

        for (int j = 0; j < sketch_->getStroke(i)->numControlPoints(); ++j)
        {
            Eigen::Vector3f prj = projectOnSketchPlane(sketch_->getStroke(i)->getControlPoint(j));
            prj_stroke_pts[i].push_back(prj);
            stroke_infos[i].max_alt = std::max(prj[2], stroke_infos[i].max_alt);
            stroke_infos[i].min_alt = std::min(prj[2], stroke_infos[i].min_alt);

            if (j == 0)
                stroke_infos[i].xmin = prj[0];
            else if (j == sketch_->getStroke(i)->numControlPoints()-1)
                stroke_infos[i].xmax = prj[0];
        }
        
        //std::cout << "Min : " << i << " = " << stroke_infos[i].min_alt << std::endl << std::flush;
    }

    for (unsigned int k = 0; k < prj_stroke_pts.size(); ++k)
    {
        for (int e = 0; e <=1; ++e)
        {
            int pos = (e==0)?0:sketch_->getStroke(k)->numControlPoints()-1;
            Eigen::Vector3f endpoint = prj_stroke_pts[k][pos];

            Eigen::Vector3f dir;
            if (pos == 0) dir = prj_stroke_pts[k][pos+1]-prj_stroke_pts[k][pos];
            else dir = prj_stroke_pts[k][pos-1]-prj_stroke_pts[k][pos];

            for (unsigned int i = 0; i < prj_stroke_pts.size(); ++i)
            {
                if ( i == k)    continue;
                float thres = (prj_stroke_pts[i][0] - prj_stroke_pts[i][1]).norm();

                for (int j = 0; j < ((int)prj_stroke_pts[i].size())-1; ++j)
                {
                    Eigen::Vector3f pt1 = prj_stroke_pts[i][j];
                    Eigen::Vector3f pt2 = prj_stroke_pts[i][j+1];
                    if (pt1[0] > pt2[0]) std::swap(pt1, pt2);

                    float t, s;

                    if (intersectLineWithSegment(endpoint, dir.normalized(), pt1, pt2, t, s) && std::abs(t) < thres*2)
                    {
                        bool isOccluded;
                        Eigen::Vector3f tangent = pt2 - pt1;

                        //Ref: http://stackoverflow.com/questions/5188561/signed-angle-between-two-3d-vectors-with-same-origin-within-the-same-plane-reci
                        Eigen::Vector3f cross = dir.cross(tangent);
                        float sign = -sketch_->camera_info().direction.normalized().dot(cross);

                        if (sign < -FLT_EPSILON) isOccluded = true;
                        else isOccluded = false;

                        stroke_infos[k].junctions[i] = isOccluded;
                        stroke_infos[i].junctions[k] = !isOccluded;

                        //fprintf(stdout, "is occluded? %d %d %d, %f\n", k, i, isOccluded, sign);
                        //fflush(stdout);
                        break;
                    }
                }
            }
        }
    }
    //std::sort(stroke_infos.begin(), stroke_infos.end());

    std::sort(stroke_infos.begin(), stroke_infos.end(), lessXAxis);
    std::vector<StrokeInfo> sorted;

    for (unsigned int i = 0; i < stroke_infos.size(); ++i)
    {
        int nprocessed = sorted.size();

            //insert new silhouette
            int l = 0, r = sorted.size();
            while (r-l > 0)
            {
                int mid = l + (r-l)/2;
                if (!(sorted[mid] < stroke_infos[i]))
                    r = mid;
                else
                    l = mid+1;
            }

            if (l >= nprocessed)
              sorted.insert(sorted.end(), stroke_infos[i]);
            else if (!(sorted[l] < stroke_infos[i]))
            {
                sorted.insert(sorted.begin()+l, stroke_infos[i]);
            }
            else
            {
                sorted.insert(sorted.begin()+l+1, stroke_infos[i]);
            }
    }

    //sorted = stroke_infos;


    for (int i = 0; i < sketch_->numStrokes(); ++i)
    {
        sorted_stroke_ids_.push_back(sorted[i].stroke_id);
        //std::cout << "i " << i << " " << stroke_infos[i].stroke_id << " " <<  stroke_infos[i].min_alt << std::endl << std::flush;
        //fprintf(stdout, " %d", stroke_infos[i].stroke_id);
    }
}

Eigen::Vector3f ConversionTo3D::projectOnSketchPlane(Eigen::Vector3f point)
{
    Eigen::Vector3f point_on_sketch = sketch_->getStroke(0)->getControlPoint(0);

    Eigen::Vector3f ray_dir = (point - sketch_->camera_info().position).normalized();
    return  projectOnPlane(point_on_sketch, -sketch_->camera_info().direction,
                           sketch_->camera_info().position, ray_dir);
}

void ConversionTo3D::computeXaxisSweeping()
{
    std::vector<SilEndpoint> endpoints;

    //Trier par abcisse croissante les extrémités gauches et droites
    for (unsigned int i = 0; i < silhouette_endpoint_pairs_.size(); ++i)
    {
        endpoints.push_back(silhouette_endpoint_pairs_[i].first);
        endpoints.push_back(silhouette_endpoint_pairs_[i].second);
    }
    std::sort(endpoints.begin(), endpoints.end());

    std::vector<SilEndpoint> processed_stroke_start;

    //Identify hidden endpoints
    for (unsigned int i = 0; i < endpoints.size(); ++i )
    {
        float min_dist = FLT_MAX;
        int tjunction_stroke_id = -1;
        for (int j = 0; j < sketch_->numStrokes(); ++j )
        {
            if (endpoints[i].stroke_id == j)
                continue;

            Stroke *stroke = sketch_->getStroke(j);
            for (int k = 0; k < stroke->numControlPoints()-1; ++k)
            {
                Stroke::Point point1 = stroke->getControlPoint(k);
                Stroke::Point point2 = stroke->getControlPoint(k+1);
                Stroke::Point point = projectOnSegment(point1, point2,
                                                       endpoints[i].pos);
                float dist = (point-endpoints[i].pos).norm();
                if (dist < min_dist)
                {
                    min_dist = dist;
                    tjunction_stroke_id = j;
                }
            }
        }

        if (min_dist < 3.0f)
        {
            endpoints[i].tjunction_stroke_id = tjunction_stroke_id;
            SilEndpointPair& endpoint_pair = silhouette_endpoint_pairs_[endpoints[i].stroke_id];
            if (endpoints[i].stroke_point_id == 0)
                endpoint_pair.first.tjunction_stroke_id = tjunction_stroke_id;
            else
                endpoint_pair.second.tjunction_stroke_id = tjunction_stroke_id;
        }
    }

    //Balayage de gauche a droite
    for (unsigned int i = 0; i < endpoints.size(); ++i)
    {
        SilEndpoint endpoint = endpoints[i];
        int nprocessed = processed_stroke_start.size();

        //insert new silhouette
        if (endpoint.stroke_point_id == 0) //is start point
        {
            int l = 0, r = processed_stroke_start.size();
            while (r-l > 0)
            {
                int mid = l + (r-l)/2;
                if (aboveStroke(processed_stroke_start[mid], endpoint))
                    r = mid;
                else
                    l = mid+1;
            }

            if (l >= nprocessed)
                processed_stroke_start.insert(processed_stroke_start.end(),
                                              endpoint);
            else if (aboveStroke(processed_stroke_start[l], endpoint))
            {
                processed_stroke_start.insert(processed_stroke_start.begin()+l,
                                              endpoint);
            }
            else
            {
                processed_stroke_start.insert(processed_stroke_start.begin()+l+1,
                                              endpoint);
            }
        }

        //label endpoint
        if (endpoint.tjunction_stroke_id >= 0)
        {
            if (isHiddenEndpoint(endpoint))
                endpoint.label = SilEndPointLabel(endpoint.tjunction_stroke_id,
                                                  kRelOccludedBy);
            else
                endpoint.label = SilEndPointLabel(endpoint.tjunction_stroke_id,
                                                  kRelTripleJunction);
        } else
        {
            //Lies on whatever stroke is behind it
            endpoint.label = SilEndPointLabel(-1, kRelLiesOn);
        }

        SilEndpointPair& endpoint_pair = silhouette_endpoint_pairs_[endpoint.stroke_id];
        if (endpoint.stroke_point_id == 0)
            endpoint_pair.first.label = endpoint.label;
        else
            endpoint_pair.second.label = endpoint.label;
    }

    sorted_stroke_ids_.clear();
    for (unsigned int i = 0; i < processed_stroke_start.size(); ++i)
    {
        int stroke_id = processed_stroke_start[i].stroke_id;
        SilEndpointPair& endpoint_pair = silhouette_endpoint_pairs_[stroke_id];

        if (endpoint_pair.first.label.ltype == kRelLiesOn)
        {
            if (i == processed_stroke_start.size()-1)
                endpoint_pair.first.label.ltype = kRelNone;
            else
                endpoint_pair.first.label.other_stroke_id = processed_stroke_start[i+1].stroke_id;
        }
        if (endpoint_pair.second.label.ltype == kRelLiesOn)
        {
            if (i == processed_stroke_start.size()-1)
                endpoint_pair.second.label.ltype = kRelNone;
            else
                endpoint_pair.second.label.other_stroke_id = processed_stroke_start[i+1].stroke_id;
        }

        sorted_stroke_ids_.push_back(stroke_id);
    }

}

void ConversionTo3D::createSilhouetteEndpointPairs()
{
    silhouette_endpoint_pairs_.resize(sketch_->numStrokes());
    silhouette_max_alts_.resize(sketch_->numStrokes(), -FLT_MAX);

    for (int i = 0; i < sketch_->numStrokes(); ++i )
    {
        Stroke *stroke = sketch_->getStroke(i);
        for (int j = 0; j < stroke->numControlPoints(); ++j)
            silhouette_max_alts_[i] = std::max(silhouette_max_alts_[i],
                                               stroke->getControlPoint(j)[2]);

        SilEndpointPair endpoint_pair;
        endpoint_pair.first.stroke_id = i;
        endpoint_pair.first.stroke_point_id = 0;
        endpoint_pair.first.pos = stroke->getControlPoint(0);
        endpoint_pair.second.stroke_id = i;
        endpoint_pair.second.stroke_point_id = stroke->numControlPoints()-1;
        endpoint_pair.second.pos = stroke->getControlPoint(stroke->numControlPoints()-1);
        silhouette_endpoint_pairs_[i] = endpoint_pair;
    }
}

void ConversionTo3D::applyFarawayProjection()
{
    int n = sorted_stroke_ids_.size();

    for (int i = n-2; i >= 0; --i)
    {
        Stroke *stroke = sketch_->getStroke(sorted_stroke_ids_[i]);
        std::vector<Eigen::Vector3f> projected_on_stroke;
        std::vector<Eigen::Vector3f> projected_on_terrain;

        projectSilhouetteOnSilhouette(stroke,
                                      sketch_->getStroke(sorted_stroke_ids_[i+1]),
                                      projected_on_stroke);
        projectSilhouetteOnFlatTerrain(stroke,
                                       projected_on_terrain);

        float dist_to_projected = (projected_on_stroke[0]-sketch_->camera_info().position).norm();
        float dist_to_unprojected = (projected_on_terrain[0]-sketch_->camera_info().position).norm();

        if (dist_to_projected < dist_to_unprojected)
            for (int j = 0; j < stroke->numControlPoints(); ++j)
                stroke->setControlPoint(j, projected_on_stroke[j]);
        else
            for (int j = 0; j < stroke->numControlPoints(); ++j)
                stroke->setControlPoint(j, projected_on_terrain[j]);
    }
}

void ConversionTo3D::applyNearerAdjustment()
{

}

bool ConversionTo3D::isHiddenEndpoint(const SilEndpoint &endpoint)
{
    if (endpoint.tjunction_stroke_id < 0)
        return false;

    Eigen::Vector3f next_point, v;
    int stroke_id = endpoint.stroke_id;
    Stroke *stroke = sketch_->getStroke(stroke_id);
    if (endpoint.stroke_point_id == 0)
    {
        int k = (endpoint.stroke_point_id+3 < stroke->numControlPoints())?3:(stroke->numControlPoints()-1-endpoint.stroke_point_id);
        next_point = stroke->getControlPoint(endpoint.stroke_point_id+k);
    }  else
    {
        int k = (endpoint.stroke_point_id-3 >= 0)?3:(endpoint.stroke_point_id);
        next_point = stroke->getControlPoint(endpoint.stroke_point_id-k);
    }

    v = next_point-endpoint.pos;
    v = v.normalized();
    if (v.dot(Eigen::Vector3f(0, 0, 1)) >= -FLT_EPSILON)
        return true;
    else
        return false;
}

bool ConversionTo3D::aboveStroke(const SilEndpoint &endpoint1,
                                 const SilEndpoint &endpoint2)
{
    SilEndpoint start1 = silhouette_endpoint_pairs_[endpoint1.stroke_id].first;
    SilEndpoint end1 = silhouette_endpoint_pairs_[endpoint1.stroke_id].second;

    SilEndpoint start2 = silhouette_endpoint_pairs_[endpoint2.stroke_id].first;
    SilEndpoint end2 = silhouette_endpoint_pairs_[endpoint2.stroke_id].second;

    if (start1.tjunction_stroke_id == endpoint2.stroke_id)
    {
        if (isHiddenEndpoint(start1))
            return true;
        else return false;
    } else if (end1.tjunction_stroke_id == endpoint2.stroke_id)
    {
        if (isHiddenEndpoint(end1))
            return true;
        else return false;
    }
    else if  (start2.tjunction_stroke_id == endpoint1.stroke_id)
    {
        if (isHiddenEndpoint(start2))
            return false;
        else return true;
    }
    else if  (end2.tjunction_stroke_id == endpoint1.stroke_id)
    {
        if (isHiddenEndpoint(end2))
            return false;
        else return true;
    }

    return silhouette_max_alts_[start1.stroke_id]  >  silhouette_max_alts_[start2.stroke_id];

    //    int stroke_id1 = start1.stroke_id;
    //    SilEndpoint end1 = silhouette_endpoint_pairs_[stroke_id1].second;
    //    return (end1.pos[2] > start2.pos[2]); //TODO This does not work in most situations
}

void ConversionTo3D::projectSilhouetteOnSilhouette(Stroke* stroke, Stroke* on_stroke,
                                                   std::vector<Eigen::Vector3f> &projected)
{
    Paraboloid paraboloid = getParaboloidForStroke(on_stroke);
    projectSilhouetteOnParaboloid(stroke, paraboloid, projected);
}

void ConversionTo3D::projectSilhouetteOnParaboloid(Stroke* stroke, Paraboloid on_paraboloid,
                                                   std::vector<Eigen::Vector3f> &projected)
{

}

void ConversionTo3D::projectSilhouetteOnFlatTerrain(Stroke *stroke,
                                                    std::vector<Eigen::Vector3f> &projected)
{
    //TODO Currently assumes up_vector of camera is (0, 0, 1)
    projected.clear();
    for (int j = 0; j < stroke->numControlPoints(); ++j)
        projected.push_back(stroke->getControlPoint(j));

}

ConversionTo3D::Paraboloid ConversionTo3D::getParaboloidForStroke(Stroke *stroke)
{
	return ConversionTo3D::Paraboloid();
}
