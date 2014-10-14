#ifndef CAMERA_INFO_H
#define CAMERA_INFO_H

#include <vector>
#include <eigen3/Eigen/Core>

struct CameraInfo
{
    Eigen::Vector3f position;
    Eigen::Vector3f direction;
    Eigen::Vector3f up_vector;
    int screen_width, screen_height;
    float fov_in_rads;
    bool is_orthographic;
    float zNear, zFar;

    CameraInfo();

    CameraInfo(Eigen::Vector3f _position,
               Eigen::Vector3f _direction,
               Eigen::Vector3f _up_vector,
               int _screen_width,
               int _screen_height,
               float _fov_in_rads  = 0.785398,
               float _zNear = 0.01f,
               float _zFar = 2.0f);

    float aspect_ratio() const { return ((float) screen_width)/screen_height; }

    void write_text(std::ostream &os);
    void read_text(std::istream &is);
};

Eigen::Vector3f projectOnPlane(Eigen::Vector3f point_on_plane,
                               Eigen::Vector3f plane_normal,
                               Eigen::Vector3f ray_origin,
                               Eigen::Vector3f ray_dir);

Eigen::Vector3f projectOnPlane(Eigen::Vector3f point_on_plane,
                               Eigen::Vector3f plane_normal,
                               Eigen::Vector3f point);

Eigen::Vector3f projectOnSegment(Eigen::Vector3f from_point,
                                 Eigen::Vector3f to_point,
                                 Eigen::Vector3f point);

bool intersectLineWithLine(Eigen::Vector3f p1, Eigen::Vector3f dir1 ,
                           Eigen::Vector3f p2, Eigen::Vector3f dir2,
                           float &t, float &s);

bool intersectLineWithSegment(Eigen::Vector3f start, Eigen::Vector3f line_dir ,
                              Eigen::Vector3f point1, Eigen::Vector3f point2,
                              float &t, float &s);

void bresenham(Eigen::Vector3f start, Eigen::Vector3f stop,
               std::vector<Eigen::Vector3f> &points);

Eigen::Vector3f heatcolor(float value, float minval=0.0, float maxval=1.0);

#endif // CAMERA_INFO_H
