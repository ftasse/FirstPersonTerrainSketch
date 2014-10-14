#ifndef STROKE_H
#define STROKE_H

#include "geometry/spline.h"

struct CameraInfo;

class Stroke : public BezierSpline
{
public:
    typedef BezierSpline::Point Point;

    Stroke();

    void adaptForTerrain(const CameraInfo &camera_info);

    void set_identifier(const std::string &identifier) { identifier_ = identifier; }
    std::string identifier() const { return identifier_; }

    int sort_id;

private:
    std::string identifier_;
};

#endif // STROKE_H
