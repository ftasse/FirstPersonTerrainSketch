#include "offline_renderer.h"

#include <stdio.h>
#include <float.h>
#include <iostream>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

OfflineRenderer::OfflineRenderer()
    : camera_info_ (Eigen::Vector3f(0.0, 0.0, 0.0),
                    Eigen::Vector3f(0.0, 1.0, 0.0),
                    Eigen::Vector3f(0.0, 0.0, 1.0),
                    256, 256),
      pixelbuffer_(NULL)
{
}

OfflineRenderer::~OfflineRenderer()
{
    delete pixelbuffer_;
    pixelbuffer_ = NULL;
}

void OfflineRenderer::setCameraInfo (const CameraInfo &camera_info)
{
    camera_info_ = camera_info;

    if (pixelbuffer_ != NULL)
    {
        pixelbuffer_->makeCurrent();
        glewInit();
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);
        glClearColor(1.0, 1.0, 1.0, 1.0);

        glViewport(0, 0, camera_info_.screen_width, camera_info_.screen_height);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(camera_info_.fov_in_rads*180/M_PI,
                       camera_info_.aspect_ratio(),
                       camera_info_.zNear, camera_info_.zFar);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        Eigen::Vector3f eye = camera_info_.position;
        Eigen::Vector3f target = eye + camera_info_.direction.normalized();
        Eigen::Vector3f up = camera_info_.up_vector;
        gluLookAt(eye[0], eye[1], eye[2],
                  target[0], target[1], target[2],
                  up[0], up[1], up[2]);

        pixelbuffer_->doneCurrent();
    }
}

bool OfflineRenderer::setupPixelBuffer()
{
    pixelbuffer_ = new QGLPixelBuffer(camera_info_.screen_width, camera_info_.screen_height);
    Q_ASSERT_X( QGLPixelBuffer::hasOpenGLPbuffers(), "DepthMapExtractor Constructor",
                "You must have a graphic card that support pbuffer OpenGL extension" );

    if (!pixelbuffer_->isValid())
    {
        fprintf(stderr, "GL Error: Could not create OpenGL context\n");
        return false;
    } else
    {
        setCameraInfo(camera_info_);
        return true;
    }
}

void OfflineRenderer::cleanupPixelBuffer()
{
    delete pixelbuffer_;
    pixelbuffer_ = NULL;
}

bool OfflineRenderer::saveBuffer(const char *path, GLenum format)
{
    if (pixelbuffer_ == NULL) return false;
    else pixelbuffer_->makeCurrent();
    bool success = false;

    QImage img(width(), height(), QImage::Format_RGB888);

    if (format == GL_RGB)
    {
        glReadPixels(0, 0, width(), height(), format, GL_UNSIGNED_BYTE, img.bits());

    } else if (format == GL_DEPTH_COMPONENT || format == GL_RED)
    {
        float *pixels = new float [width()*height()];
        glReadPixels(0, 0, width(), height(), format, GL_FLOAT, pixels);

        float max_depth = -FLT_MAX, min_depth = FLT_MAX;
        for (int y = 0; y < height(); ++y)
            for (int x = 0; x < width(); ++x)
            {
                max_depth = std::max(max_depth, pixels[y*width()+x]);
                min_depth = std::min(min_depth, pixels[y*width()+x]);
            }
        for (int y = 0; y < height(); ++y)
            for (int x = 0; x < width(); ++x)
            {
                int val = 255.0f*(pixels[y*width()+x] - min_depth)/(max_depth-min_depth);
                QRgb col = qRgb(val, val, val);
                img.setPixel(x, y, col);
            }
        delete pixels;
    }

    img = img.mirrored();
    img.save(path);
    success = true;
    return success;
}

void OfflineRenderer::updateMatrices()
{
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview_matrix_);
    glGetDoublev(GL_PROJECTION_MATRIX, projection_matrix_);
    glGetIntegerv(GL_VIEWPORT, viewport_);
}

bool OfflineRenderer::pixelToWorldCoords(Eigen::Vector2i point,
                                         Eigen::Vector3f &world_coords,
                                         bool &is_background)
{
    float srcZ = 1.0;
    double objX, objY, objZ;

    glReadPixels(point[0], height()-point[1]-1, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &srcZ);
    bool succeded = gluUnProject(point[0], height()-point[1]-1, srcZ,
                                 modelview_matrix_, projection_matrix_, viewport_,
                                 &objX, &objY, &objZ);
    if (srcZ == 1.0) is_background = true;
    else is_background = false;

    if (!succeded)
    {
        std::cerr << "Warning: could not unproject screen point at: "
                  << point[0] << " " << point[1] << std::endl << std::flush;
        return false;
    }
    else
    {
        world_coords = Eigen::Vector3f(objX, objY, objZ);
        return true;
    }
}
