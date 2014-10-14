#include "stroke.h"

#include <float.h>
#include "tools/camera_info.h"

#include <Eigen/Geometry>

Stroke::Stroke():
    BezierSpline()
{
}

void Stroke::adaptForTerrain(const CameraInfo &camera_info)
{
  // Project on a plane orthogonal to XY Plane
    for (int i = 0; i < numControlPoints(); ++i)
    {
        Stroke::Point projected;
        Stroke::Point point = getControlPoint(i);
        projected = projectOnPlane(Eigen::Vector3f(0.0, 0.0, 0.0),
                                   Eigen::Vector3f(0.0, 0.0, 1.0),
                                   point);
        projected[2] = point[2];
        setControlPoint(i, projected);
    }
}


