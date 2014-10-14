#ifndef BEZIER_CURVE_H
#define BEZIER_CURVE_H

#include "geometry/polyline.h"

using namespace TerrSketch;

/** brief Bezier Curve
  *
**/
class BezierSpline
{
public:
    typedef Polyline::Point Point;

    BezierSpline();
    ~BezierSpline();

    bool closed() const { return closed_; }
    void setClosed(bool closed) { closed_ = closed; }

    int degree() const { return degree_;}
    int order() const { return degree() + 1; }

    void addControlPoint(const Point &point);
    Point getControlPoint(int idx) const;
    int numControlPoints() const;
    void clear() { control_points_.clear(); knots_.clear(); }

    void updateKnots();
    int numKnots() const { return knots_.size(); }
    float* knotsPtr() { if (numKnots() > 0) return &knots_[0]; else return NULL ; }

    void convertFromPolyline(const TerrSketch::Polyline &polyline, int smooth_coefficient = 3);

    void displaceHeight(float hdiff);

    void write_text(std::ostream &os, int num_points = 0);
    void read_text(std::istream &is, int num_points);
    
    
    std::vector<Point> getPoints() const { return control_points_; }
    void setControlPoint(int idx, const Point &point);

    void invert()
    {
        std::reverse(control_points_.begin(), control_points_.end());
    }

    void setPoints(std::vector<Eigen::Vector3f> &points)
    {
        control_points_ = points;
    }

    void getPoints(std::vector<Eigen::Vector3f> &points)
    {
        points  = control_points_;
    }

    void cleanup();

    void subSample(float tol);
    void simplify(float min_distance, float max_distance);

protected:

private:
    std::vector<Point> control_points_;
    std::vector<float> knots_;
    bool closed_;
    int degree_;
};

#endif // BEZIER_CURVE_H
