#include "polyline_completion.h"
#include "tools/camera_info.h"

#include <float.h>
#include <iostream>

#include "geometry/terrain.h"
#include "tools/math_extras.h"

PolylineCompletion::PolylineCompletion()
{
}

void PolylineCompletion::completePolyline(std::vector<Eigen::Vector3f> &in_points,
                                          std::vector<Eigen::Vector3f> &out_points,
                                          Eigen::Vector3f end_point,
                                          bool isLeft, bool sampled)//Eigen::Vector3f right_most_point
{
    std::vector<Eigen::Vector3f> points;
    end_point[0] = round(end_point[0]);
    end_point[1] = round(end_point[1]);

    out_points.clear();
    out_points.insert(out_points.end(), in_points.begin(), in_points.end());
    if (isLeft)
    {
        if (!sampled)
          out_points.insert(out_points.begin(), end_point);
        else
          {
        Eigen::Vector3f t = (end_point-in_points.front());
        float step = 3*(in_points[1]-in_points.front()).norm();
        int n = round(t.norm()/step);
        t = t/n;
        for (int k=0; k < n; ++k)
            points.push_back(end_point - k*t);

        out_points.insert(out_points.begin(), points.begin(), points.end());
          }
    }
    else
    {
        if (!sampled)
          out_points.insert(out_points.end(), end_point);
        else
          {
        Eigen::Vector3f t = (in_points.back() - end_point);
        float step = 3*(in_points.back()-in_points[in_points.size()-2]).norm();
        int n = round(t.norm()/step);
        t = t/n;
        for (int k=1; k <= n; ++k)
            points.push_back(in_points.back() - k*t);

        out_points.insert(out_points.end(), points.begin(), points.end());
          }
    }
}

void PolylineCompletion::completePolyline(std::vector<Eigen::Vector3f> &in_points,
                                          std::vector<Eigen::Vector3f> &out_points,
                                          Eigen::Vector3f min_point)
{
    Eigen::Vector3f plane_normal(0, 0, 1);
    Eigen::Vector3f left_most_point, right_most_point;
    std::vector<Eigen::Vector3f> tmp  = in_points;

    if (-(in_points[2] - in_points[0])[2] < 0.0)
    {
        left_most_point = projectOnPlane(min_point, plane_normal, in_points.front(),
                                         -(in_points[2] - in_points[0]).normalized());
        completePolyline(tmp, out_points, left_most_point, true);
        tmp = out_points;
    }

    if (-(in_points[in_points.size() - 3] - in_points.back())[2] < 0.0)
    {
        right_most_point = projectOnPlane(min_point, plane_normal, in_points.back(),
                                         -(in_points[in_points.size() - 3] - in_points.back()).normalized());

        completePolyline(tmp, out_points, right_most_point, false);
        tmp = out_points;
    }
}

void PolylineCompletion::completePolyline(std::vector<Eigen::Vector3f> &in_points,
                                          std::vector<Eigen::Vector3f> &out_points,
                                          Terrain *terrain)
{
  Eigen::Vector3f left_most_point, right_most_point;
  std::vector<Eigen::Vector3f> tmp  = in_points;

  //if (-(in_points[2] - in_points[0])[2] < 0.0)
  {
      //left_most_point = projectOnPlane(min_point, plane_normal, in_points.front(),-(in_points[2] - in_points[0]).normalized());

      std::vector<Eigen::Vector3f> intersects;
      bool success = rayTerrainIntersect(terrain, in_points.front(),
                                         -(in_points[4] - in_points[0]).normalized(),
                                         terrain->width(), intersects, 0.01);
      if (success)
      {
          left_most_point = intersects.front();

          std::cout << "left " << left_most_point[2] << " " << terrain->getInterpolatedAltitude(left_most_point[0], left_most_point[1])
              << std::endl << std::flush;

          completePolyline(tmp, out_points, left_most_point, true);
          tmp = out_points;
      }
  }

  //if (-(in_points[in_points.size() - 3] - in_points.back())[2] < 0.0)
  {
      //right_most_point = projectOnPlane(min_point, plane_normal, in_points.back(), -(in_points[in_points.size() - 3] - in_points.back()).normalized());

      std::vector<Eigen::Vector3f> intersects;
      bool success = rayTerrainIntersect(terrain, in_points.back(),
                                         -(in_points[in_points.size() - 5] - in_points.back()).normalized(),
                                          terrain->width(), intersects, 0.01);
      if (success)
      {
          right_most_point = intersects.front();

          std::cout << "right " << right_most_point[2] << " " << terrain->getInterpolatedAltitude(right_most_point[0], right_most_point[1])
              << std::endl << std::flush;

          completePolyline(tmp, out_points, right_most_point, false);
          tmp = out_points;
      }
  }
}

void PolylineCompletion::completePolyline(std::vector<Eigen::Vector3f> &in_points,
                                          std::vector<Eigen::Vector3f> &out_points)
{
    Eigen::Vector3f min_point; min_point[2] = FLT_MAX;

    for (unsigned int i=0; i<in_points.size(); ++i)
    {
        if (in_points[i][2] < min_point[2])
            min_point = in_points[i];
    }

    completePolyline(in_points, out_points, min_point);
}

void PolylineCompletion::completePolylines()
{
    completed_polylines_.clear();

    Eigen::Vector3f min_point; min_point[2] = FLT_MAX;

    for (unsigned int i=0; i<polylines_.size(); ++i)
    {
        for (unsigned int j=0; j < polylines_[i].size(); ++j)
            if (polylines_[i][j][2] < min_point[2])
                min_point = polylines_[i][j];
    }

    completed_polylines_.resize(polylines_.size());
    for (unsigned int i=0; i<polylines_.size(); ++i)
    {
        completePolyline(polylines_[i], completed_polylines_[i], min_point);
    }
}

void PolylineCompletion::completePolylines(Terrain *terrain)
{
    completed_polylines_.clear();

    completed_polylines_.resize(polylines_.size());
    for (unsigned int i=0; i<polylines_.size(); ++i)
    {

        completePolyline(polylines_[i], completed_polylines_[i], terrain);

    }
}
