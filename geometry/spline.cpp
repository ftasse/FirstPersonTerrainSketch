#include "spline.h"

#include <float.h>

/*
  Bezier Spline
*/

BezierSpline::BezierSpline():
    closed_(false), degree_(3)
{
}

BezierSpline::~BezierSpline()
{

}

void BezierSpline::displaceHeight(float hdiff)
{
  for (unsigned int i = 0; i < numControlPoints(); ++i)
    {
      control_points_[i][2] += hdiff;
    }
}

void BezierSpline::subSample(float tol)
{
    if (numControlPoints() <= 2)
        return;

    std::vector<Point> new_points;
    ::subSample(control_points_, new_points, tol);
    control_points_ = new_points;
}

void BezierSpline::simplify(float min_distance, float max_distance)
{
    if (numControlPoints() <= 2)
        return;

    std::vector<Point> sample_points;
    ::simplify(control_points_, sample_points, min_distance, max_distance);
    control_points_ = sample_points;
}

void BezierSpline::convertFromPolyline(const Polyline &polyline, int smooth_coefficient)
{
    float scale = 1.0/smooth_coefficient;
    int n = polyline.numPoints();

    assert(n >= 2);

    for (int i = 0; i < n; ++i)
        addControlPoint(polyline.getPoint(i));

//    for (int i = 0; i < n; ++i)
//    {
//        if (i == 0) // is first
//        {
//            Eigen::Vector3f p1 = polyline.getPoint(i);
//            Eigen::Vector3f p2 = polyline.getPoint(i+1);

//            Eigen::Vector3f tangent = (p2 - p1);
//            Eigen::Vector3f q1 = p1 + scale * tangent;

//            addControlPoint(p1);
//            addControlPoint(q1);
//        } else if (i == n - 1)
//        {
//            Eigen::Vector3f p0 = polyline.getPoint(i-1);
//            Eigen::Vector3f p1 = polyline.getPoint(i);
//            Eigen::Vector3f tangent = (p1 - p0);
//            Eigen::Vector3f q0 = p1 - scale * tangent;

//            addControlPoint(q0);
//            addControlPoint(p1);
//        } else
//        {
//            Eigen::Vector3f p0 = polyline.getPoint(i-1);
//            Eigen::Vector3f p1 = polyline.getPoint(i);
//            Eigen::Vector3f p2 = polyline.getPoint(i+1);
//            Eigen::Vector3f tangent = (p2 - p0).normalized();
//            Eigen::Vector3f q0 = p1 - scale * tangent * (p1 - p0).norm();
//            Eigen::Vector3f q1 = p1 + scale * tangent * (p2 - p1).norm();

//            addControlPoint(q0);
//            addControlPoint(p1);
//            addControlPoint(q1);
//        }
//    }

    setClosed(polyline.closed());
}

void BezierSpline::addControlPoint(const Point &point)
{
    control_points_.push_back(point);
}

void BezierSpline::setControlPoint(int idx, const Point &point)
{
  assert(idx >= 0 && idx < numControlPoints());
  control_points_[idx] = point;
}

BezierSpline::Point BezierSpline::getControlPoint(int idx) const
{
    assert(idx >= 0 && idx < numControlPoints());
    return control_points_[idx];
}

int BezierSpline::numControlPoints() const
{
    return control_points_.size();
}

void BezierSpline::cleanup()
{
    int n = numControlPoints();
    if (n > 1)
    {
        for (int i = n-1; i >= 1; --i)
            if ((control_points_[i] - control_points_[i-1]).norm() < FLT_EPSILON)
                control_points_.erase(control_points_.begin()+i);
    }
}

void BezierSpline::updateKnots()
{
    //assert(numControlPoints() >= order());

    knots_.resize(numControlPoints() + order());
    for (unsigned int i = 0; i < order(); ++i)
        knots_[i] = 0.0f;

    for (unsigned int i = order(); i < numKnots()-order(); ++i)
        knots_[i] = (i-order() + 1) / ((float) numKnots()-2*order() + 1);

    for (unsigned int i = numKnots()-order(); i < numKnots(); ++i)
        knots_[i] = 1.0f;
}

void BezierSpline::write_text(std::ostream &os, int num_points)
{
    if (num_points == 0)
        num_points = (closed())?numControlPoints()+1:numControlPoints();

    for (int j = 0; j < num_points; ++j)
    {
        Point point = getControlPoint(j % numControlPoints());
        os << point[0] << " "  << point[1] << " "  << point[2] << " ";
    }
    os << std::endl;
}

void BezierSpline::read_text(std::istream &is, int num_points)
{

    for (int j = 0; j < num_points; ++j)
    {
        Point point;
        is >> point[0] >> point[1] >> point[2];

        if (j == num_points-1 && (point-getControlPoint(0)).norm() < FLT_EPSILON)
            setClosed(true);
        else
            addControlPoint(point);
    }
}
