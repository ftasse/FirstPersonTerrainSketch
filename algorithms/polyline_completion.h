#ifndef POLYLINE_COMPLETION_H
#define POLYLINE_COMPLETION_H

#include <vector>
#include <Eigen/Core>

class Terrain;

class PolylineCompletion
{
public:
    PolylineCompletion();

    void completePolyline(std::vector<Eigen::Vector3f> &in_points,
                          std::vector<Eigen::Vector3f> &out_points);

    void completePolyline(std::vector<Eigen::Vector3f> &in_points,
                          std::vector<Eigen::Vector3f> &out_points,
                          Eigen::Vector3f min_point);

    void completePolyline(std::vector<Eigen::Vector3f> &in_points,
                          std::vector<Eigen::Vector3f> &out_points,
                          Terrain *terrain);

    void completePolyline(std::vector<Eigen::Vector3f> &in_points,
                          std::vector<Eigen::Vector3f> &out_points,
                          Eigen::Vector3f end_point,
                          bool isLeft, bool sampled = true);

    void completePolylines(Terrain *terrain);

    void getCompletedPolylines ( std::vector< std::vector<Eigen::Vector3f> > &res) {
        res = completed_polylines_;
    }

    void setPolylines ( std::vector< std::vector<Eigen::Vector3f> > &res) {
        polylines_ = res;
    }

    void completePolylines();

private:
    std::vector< std::vector<Eigen::Vector3f> > polylines_;
    std::vector< std::vector<Eigen::Vector3f> > completed_polylines_;
};

#endif // POLYLINE_COMPLETION_H
