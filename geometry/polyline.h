#ifndef POLYLINE_H
#define POLYLINE_H

#include <vector>
#include <eigen3/Eigen/Core>

#include "tools/math_extras.h"

namespace TerrSketch {

class Polyline
{
public:
    typedef Eigen::Vector3f Point;
    typedef std::pair<int, int> EdgeInfo;
    typedef std::pair<Point, Point> Edge;

    Polyline();

    void addPoint(const Point &point);
    Point getPoint(int idx) const;
    int numPoints() const;

    bool closed() const { return closed_; }
    void setClosed(bool closed) { closed_ = closed; }

    int numEdges() const;
    EdgeInfo getEdgeInfo(int idx);
    Edge getEdge(int idx);

    void subSample(float tol);
    void simplify(float min_distance, float max_distance);
    void simplifyDouglasPeucker(float eps = 1e-5);

    void displaceHeight(float hdiff);

private:
    std::vector<Point> points_;
    bool closed_;
};

void subSample(std::vector<Polyline::Point> &points,
               std::vector<Polyline::Point> &sample,
               float tol);

void simplify(std::vector<Polyline::Point> &points,
              std::vector<Polyline::Point> &sample,
              float min_distance, float max_distance);

void simplifyDouglasPeucker(std::vector<Polyline::Point> &points,
                            std::vector<Polyline::Point> &sample,
                            float eps = 1e-5);
}

#endif // POLYLINE_H
