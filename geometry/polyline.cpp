#include "polyline.h"

#include <float.h>

namespace TerrSketch {

Polyline::Polyline():
    closed_(false)
{
}


void Polyline::addPoint(const Point &point)
{
    points_.push_back(point);
}

Polyline::Point Polyline::getPoint(int idx) const
{
    assert(idx >= 0 && idx < numPoints());
    return points_[idx];
}

int Polyline::numPoints() const
{
    return points_.size();
}

int Polyline::numEdges() const
{
    if (closed())
        return numPoints()-1;
    else
        return numPoints();
}

Polyline::EdgeInfo Polyline::getEdgeInfo(int idx)
{
    assert(idx >= 0 && idx < numEdges());
    return EdgeInfo(idx, (idx+1)%numPoints());
}

Polyline::Edge Polyline::getEdge(int idx)
{
    EdgeInfo info = getEdgeInfo(idx);
    return Edge(getPoint(info.first), getPoint(info.second));
}

void Polyline::subSample(float tol)
{
    if (numPoints() <= 2)
        return;

    std::vector<Point> new_points;
    TerrSketch::subSample(points_, new_points, tol);
    points_ = new_points;
}

void Polyline::simplify(float min_distance, float max_distance)
{
    if (numPoints() <= 2)
        return;

    std::vector<Point> sample_points;
    TerrSketch::simplify(points_, sample_points, min_distance, max_distance);
    points_ = sample_points;
}

void Polyline::simplifyDouglasPeucker(float eps)
{
    if (numPoints() <= 2)
        return;

    std::vector<Point> sample_points;
    TerrSketch::simplifyDouglasPeucker(points_, sample_points, eps);
    points_ = sample_points;
}

void subSample(std::vector<Polyline::Point> &points,
               std::vector<Polyline::Point> &sample, float tol)
{
    int n = points.size();
    sample.push_back(points.front());

    if (n <= 1)
        return;

    for(int i = 0; i < n - 1; i++)
    {
        Eigen::Vector3f edge = points[i+1] - points[i];
        float len = edge.norm();

        if (len > tol)
        {
            int n = floor(0.5 + len/tol);
            for (int j = 1; j < n; ++j)
            {
                sample.push_back(points[i] + (j*1.0f/n)*edge);
            }
        }
        sample.push_back(points[i+1]);
    }
}

void simplify(std::vector<Polyline::Point> &points,
              std::vector<Polyline::Point> &sample,
              float min_distance, float max_distance)
{
    sample.clear();
    int n = points.size();
    if (n <= 2)
    {
        sample = points;
        return;
    }

    sample.push_back(points[0]);
    Polyline::Point potential_sample_point = points[1];

    for (int i = 2; i < n; i++)
    {
        if(((potential_sample_point - points[i]).norm() > min_distance) &&
                ((sample.back() - points[i]).norm() > max_distance))
        {
            sample.push_back(potential_sample_point);
        }

        potential_sample_point = points[i];
    }

    //now handle last bit of curve
    Polyline::Point p1 = sample.back(); sample.pop_back(); //last sample point
    Polyline::Point p0 = sample.back(); //second last sample point
    Eigen::Vector3f tangent = (p0 - potential_sample_point).normalized();
    float d2 = (potential_sample_point - p1).norm();
    float d1 = (p1 - p0).norm();
    p1 = p1 + tangent * ((d1 - d2)/2);

    sample.push_back(p1);
    sample.push_back(potential_sample_point);
}

void simplifyDouglasPeucker(std::vector<Polyline::Point> &points,
                            std::vector<Polyline::Point> &sample,
                            float eps)
{
    float dmax = 0;
    int index = 0;
    int n = points.size();

    if (n <= 2)
    {
        sample = points;
        return;
    }

    std::pair<Polyline::Point, Polyline::Point> line(points.front(), points.back());
    Eigen::Vector3f line_dir = (line.second - line.first).normalized();

    for (int i = 1 ; i < n-1; ++i) {
        Eigen::Vector3f v = line.first-points[i];
        float d = (v - (v.dot(line_dir))*line_dir).norm();
        if ( d > dmax ) {
            index = i;
            dmax = d;
        }
    }

    if ( dmax > eps ) {
        // Recursive call
        std::vector<Polyline::Point>  sample1, sample2;
        std::vector<Polyline::Point> points1(points.begin(), points.begin()+index);
        std::vector<Polyline::Point> points2(points.begin()+index, points.end());
        simplifyDouglasPeucker(points1, sample1, eps);
        simplifyDouglasPeucker(points2, sample2, eps);
        sample = sample1;
        sample.insert(sample.end(), sample2.begin(), sample2.end());
        } else {
            sample.push_back(points.front());
            sample.push_back(points.back());
        }
}

}