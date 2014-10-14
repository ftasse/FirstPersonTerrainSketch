#ifndef OFFLINE_RENDERER_H
#define OFFLINE_RENDERER_H

#include <GL/glew.h>
#include <QtOpenGL/QGLPixelBuffer>

#include "tools/camera_info.h"

class OfflineRenderer
{
public:
    OfflineRenderer();
    ~OfflineRenderer();

    bool saveBuffer(const char *path, GLenum format = GL_DEPTH_COMPONENT);
    void setCameraInfo (const CameraInfo &camera_info);

protected:
    bool setupPixelBuffer();
    void cleanupPixelBuffer();

    CameraInfo camera_info_;
    QGLPixelBuffer * pixelbuffer_;

    double modelview_matrix_[16];
    double projection_matrix_[16];
    int viewport_[4];

    void updateMatrices();
    int width() const  { return camera_info_.screen_width; }
    int height() const  { return camera_info_.screen_height; }

    bool pixelToWorldCoords(Eigen::Vector2i point,
                            Eigen::Vector3f &world_coords,
                            bool &is_background);
};

#endif // OFFLINE_RENDERER_H
