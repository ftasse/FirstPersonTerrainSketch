#include "tools/camera_info.h"

#include <float.h>

CameraInfo::CameraInfo():
    is_orthographic(false), fov_in_rads(0.785398),
    zNear(0.001f), zFar(2.0f)
{}

CameraInfo::CameraInfo(Eigen::Vector3f _position,
           Eigen::Vector3f _direction,
           Eigen::Vector3f _up_vector,
           int _screen_width, int _screen_height,
           float _fov_in_rads, float _zNear, float _zFar)
    : position(_position),
      direction(_direction),
      up_vector(_up_vector),
      screen_width(_screen_width),
      screen_height(_screen_height),
      fov_in_rads(_fov_in_rads),
      zNear(_zNear),
      zFar(_zFar) {}

void CameraInfo::write_text(std::ostream &os)
{
    os << "\nCameraInfo\n";
    os << "-Position: " << position[0] << " " << position[1] << " " << position[2] << std::endl;
    os << "-Direction: " << direction[0] << " " << direction[1] << " " << direction[2] << std::endl;
    os << "-UpVector: " << up_vector[0] << " " << up_vector[1] << " " << up_vector[2] << std::endl;
    os << "-ScreenSize: " << screen_width << " " << screen_height << std::endl;
    os << "-ZClipping: " << zNear << " " << zFar << std::endl;
    os << "-FieldOfView: " << fov_in_rads << std::endl;
    os << "-ORTHOGRAPHIC: " << is_orthographic << std::endl;
}

void CameraInfo::read_text(std::istream &is)
{
    std::string text;
    is >> text;
    is >> text >> position[0] >> position[1] >> position[2];
    is >> text >> direction[0] >> direction[1] >> direction[2];
    is >> text >> up_vector[0] >> up_vector[1] >> up_vector[2];
    is >> text >> screen_width >> screen_height;
    is >> text >> zNear >> zFar;
    is >> text >> fov_in_rads;
    is >> text >> is_orthographic;
}

Eigen::Vector3f projectOnPlane(Eigen::Vector3f point_on_plane,
                               Eigen::Vector3f plane_normal,
                               Eigen::Vector3f ray_origin,
                               Eigen::Vector3f ray_dir)
{
  float s = plane_normal.dot(point_on_plane - ray_origin);
  s /= plane_normal.dot(ray_dir);
  return  ray_origin + s*ray_dir;
}

Eigen::Vector3f projectOnPlane(Eigen::Vector3f point_on_plane,
                               Eigen::Vector3f plane_normal,
                               Eigen::Vector3f point)
{
    float dist_to_origin = point_on_plane.norm();
    float dist_to_plane = plane_normal.dot(point) + dist_to_origin;
    return point - dist_to_plane*plane_normal;
}

Eigen::Vector3f projectOnSegment(Eigen::Vector3f from_point,
                                 Eigen::Vector3f to_point,
                                 Eigen::Vector3f point)
{
    Eigen::Vector3f v = to_point - from_point;
    Eigen::Vector3f w = point - from_point;
    float c1 = w.dot(v);
    float c2 = v.dot(v);
    if ( c1 <= 0 ) return from_point;
    else if ( c2 <= c1 )  return to_point;

    float t = c1 / c2;
    return from_point + t*v;
}

//computes shortest line between two lines in 3D
//http://paulbourke.net/geometry/pointlineplane/
bool intersectLineWithLine(Eigen::Vector3f p1, Eigen::Vector3f dir1 ,
                           Eigen::Vector3f p2, Eigen::Vector3f dir2,
                           float &t, float &s)
{
    Eigen::Vector3f p12 = p1-p2;
    double numer,denom;

    if (dir1.norm() < FLT_EPSILON || dir2.norm() < FLT_EPSILON )
        return false;

    double d2 = p12.dot(dir2);
    double d1 = p12.dot(dir1);
    double dr21 = dir2.dot(dir1);
    double dr22 = dir2.dot(dir2);
    double dr11 = dir1.dot(dir1);

    denom = dr11 * dr22 - dr21 * dr21;
    if (std::abs(denom) < FLT_EPSILON)
        return false;
    numer = d2 * dr21 - d1 * dr22;

    t = numer / denom;
    s = (d2 + dr21 * t) / dr22;

    Eigen::Vector3f intersect_on_1 = p1 + t*dir1;
    Eigen::Vector3f intersect_on_2 = p2 + s*dir2;

    return (intersect_on_2 - intersect_on_1).norm() < 1e-4;
}

bool intersectLineWithSegment(Eigen::Vector3f start, Eigen::Vector3f line_dir ,
                              Eigen::Vector3f point1, Eigen::Vector3f point2,
                              float &t, float &s)
{
    Eigen::Vector3f seg_dir = (point2-point1);
    if (intersectLineWithLine(start, line_dir, point1, seg_dir, t, s))
        return (s >= 0.0 && s <= 1.0);
    return false;
}

void bresenham(Eigen::Vector3f start, Eigen::Vector3f stop, std::vector<Eigen::Vector3f> &points) {
    const bool steep = (fabs(stop[1] - start[1]) > fabs(stop[0] - start[0]));

    if(steep && stop[0] - start[0] > FLT_EPSILON) {
        std::swap(start[0], start[1]);
        std::swap(stop[0], stop[1]);
    }

    int deltax = stop[0] - start[0];
    int deltay = stop[1] - start[1];

    float error = 0;
    if (deltax == 0)
    {
        int ystep = (deltay > 0)?1:-1;
        for (int y = ((int)start[1]); y != ((int)stop[1]); y += ystep)
        {
            float t = (y - start[1])/deltay;
            points.push_back(Eigen::Vector3f(start[0], y,  start[2] + t*(stop[2] - start[2])));
        }
        points.push_back(stop);
    } else
    {
        float deltaerr = std::abs (deltay / ((float)deltax));    // Assume deltax != 0 (line is not vertical),
        int y = start[1];
        int xstep = (deltax > 0)?1:-1;
        int ystep = (deltay > 0)?1:-1;
        for (int x = ((int)start[0]); x != ((int)stop[0]); x += xstep)
        {
            float t = (x - start[0])/deltax;
            if (steep) points.push_back(Eigen::Vector3f(y, x, start[2] + t*(stop[2] - start[2])));
            else points.push_back(Eigen::Vector3f(x, y, start[2] + t*(stop[2] - start[2])));

            error += deltaerr;
            if (error >= 0.5) {
                y += ystep;
                error = error - 1.0;
            }
        }
        if (steep) std::swap(stop[0], stop[1]);
        points.push_back(stop);
    }
}

Eigen::Vector3f heatcolor(float value, float minval, float maxval)
{
  if ((maxval-minval) < FLT_EPSILON)  maxval += FLT_EPSILON;
    float v0 = minval + 0.0/4.0 * (maxval - minval);
    float v1 = minval + 1.0/4.0 * (maxval - minval);
    float v2 = minval + 2.0/4.0 * (maxval - minval);
    float v3 = minval + 3.0/4.0 * (maxval - minval);
    float v4 = minval + 4.0/4.0 * (maxval - minval);

    Eigen::Vector3f col(1,1,1);

        float u;

        if (value < v0)
        {
          col = Eigen::Vector3f(0, 0, 1);
        }
        else if (value > v4)
        {
          col = Eigen::Vector3f(1, 0, 0);
        }

        else if (value <= v2)
        {
          if (value <= v1) // [v0, v1]
          {
                    u = (1.0 * (value - v0) / (v1 - v0));
                    col = Eigen::Vector3f(0, u, 1);
          }
          else // ]v1, v2]
          {
                    u = (1.0 * (value - v1) / (v2 - v1));
                    col = Eigen::Vector3f(0, 1, 1-u);
          }
        }
        else
        {
          if (value <= v3) // ]v2, v3]
          {
                    u = (1.0 * (value - v2) / (v3 - v2));
                    col = Eigen::Vector3f(u, 1, 0);
          }
          else // ]v3, v4]
          {
                    u = ((value - v3) / (v4 - v3));
                    col = Eigen::Vector3f(1, 1-u, 0);
          }
        }
        return col;
}
