#ifndef TERRAIN_VIEWER_H
#define TERRAIN_VIEWER_H

#include <GL/glew.h>

#include <QGLViewer/qglviewer.h>
#include "geometry/terrain.h"
#include "tools/camera_info.h"
#include "terrain_fast_renderer.h"

class TerrainViewer : public QGLViewer
{
    Q_OBJECT

    typedef QGLViewer Base;

protected:
    typedef enum {
        kCameraFirstPersonMode = 0,
        kCameraThirdPersonMode = 1
    } CameraMode;

public:
    explicit TerrainViewer(QWidget *parent = 0);
    ~TerrainViewer();

    Terrain *getTerrain() const { return terrain_renderer_.getTerrain(); }
    bool hasTerrain() const { return getTerrain() != NULL; }

    bool loadTerrain(const char *terrain_path);
    bool saveTerrain(const char *terrain_path);
    void loadFlatTerrain(int terrain_width, int terrain_height);

    void setTerrain(Terrain *terrain, bool update_display = true);
    void setPersonHeight(float height);

    void extractSilhouetteEdges();
    void extractSilhouetteEdges(const CameraInfo &camera_info, std::vector<Terrain::SilhouetteSegment> &result);
    void extractRidges();
    void extractRidges(const CameraInfo &camera_info, std::vector<Terrain::SilhouetteSegment> &result);

    void extractFeatures(const CameraInfo &camera_info, std::vector<Terrain::SilhouetteSegment> &result);
    void extractFeatures();

    CameraMode getCameraMode() const;
    void setCameraMode(CameraMode camera_mode);

    void setWireframe(bool status)
    {
        wireframe_ = status;
        updateGL();
    }

protected:
    TerrainFastRenderer terrain_renderer_;
    bool gotSilhouettes_;
    
    bool cur_first_person_camera_locked_;

    void setCurrentCameraFromInfo(const CameraInfo &info);
    CameraInfo getCurrentCameraInfo() const;
    void updateCamera(bool update_display = true);

    Eigen::Vector3f castVec3f(const qglviewer::Vec &p) const
    {  return Eigen::Vector3f(p[0], p[1], p[2]); }

    qglviewer::Vec castVec3f(const Eigen::Vector3f &p) const
    {  return qglviewer::Vec(p[0], p[1], p[2]); }

private:
    CameraMode camera_mode_;
    bool wireframe_;
    float person_height_;

    void drawTerrain();
    void drawFirstPerson();


    void drawTerrainGridNormals();

    qglviewer::Vec getFirstPersonPosition();
    qglviewer::Vec getFirstPersonDirection();
    float getAltitudeAtWorldCoords(float fx, float fy);

protected:
    CameraInfo first_person_camerainfo_, third_person_camerainfo_;
    std::vector<Terrain::SilhouetteSegment> silhouette_segments_;

    virtual void adjustFirstPersonCamera();
    void setFirstPersonPosition(qglviewer::Vec position);

    void drawSilhouetteEdges(std::vector<Terrain::SilhouetteSegment> &segments,
                             Eigen::Vector3f edge_color);

    void init();
    void draw();

    void keyPressEvent(QKeyEvent * event);
    void wheelEvent(QWheelEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);

	QString helpString() const;

signals:
    void sendStatusMessage(QString);
    
public slots:
    void on_cameramodeCheckBox_toggled(bool status);
    void updateTerrain(bool update_person = true);
};

#endif // TERRAIN_VIEWER_H
