#include "terrain_viewer.h"

#include <math.h>
#include <iostream>
#include <fstream>

#include <QKeyEvent>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QFileDialog>
#include <eigen3/Eigen/Geometry>

#include "tools/silhouette_extractor.h"
#include "tools/feature_detection.h"

#define kCameraRotateSteps M_PI/16.0

#define kVertexPositionAttribute 0
#define kVertexNormalAttribute 1

class MyCamera : public qglviewer::Camera
{
public:
    MyCamera():
        zNear_(1.0f), zFar_(10000.0f)
    {}

    float zNear() const
    {
        return zNear_;
    }

    float zFar() const
    {
        return zFar_;
    }

    void setZNear(float zNear)
    {
        zNear_ = zNear;
    }

    void setZFar(float zFar)
    {
        zFar_ = zFar;
    }

private:
     float zNear_, zFar_;
};

TerrainViewer::TerrainViewer(QWidget *parent) :
    QGLViewer(parent),
    cur_first_person_camera_locked_(false),
    camera_mode_(kCameraThirdPersonMode),
    wireframe_(false)
{

    // Enable OpenGL multisampling 
    QGLFormat format(QGL::SampleBuffers | QGL::DoubleBuffer | QGL::AlphaChannel);  
    format.setSamples(16);
    setFormat(format);
    makeCurrent();

    person_height_ = 2;
    setCamera(new MyCamera());
}

TerrainViewer::~TerrainViewer()
{
    setTerrain(NULL, false);
    terrain_renderer_.cleanup();
}

void TerrainViewer::init()
{
    int major, minor;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);
    std::cout << "OpenGL version " << major << "." << minor << std::endl << std::flush;

    glewInit();
    terrain_renderer_.init();
    glEnable(GL_DEPTH_TEST);

    setBackgroundColor(QColor(255, 255, 255));

    camera()->setUpVector(qglviewer::Vec(0, 0, 1));
    camera()->setType(qglviewer::Camera::PERSPECTIVE);

    //First person
    camera()->setPosition(qglviewer::Vec(5, 5.0, person_height_));
    camera()->setViewDirection(qglviewer::Vec(1.0, 1.0, 0.0));
    first_person_camerainfo_ = getCurrentCameraInfo();
    first_person_camerainfo_.is_orthographic = false;

    //third person
    camera()->showEntireScene();
    //camera()->lookAt(qglviewer::Vec(0.0, 1.0, 0.0));
    third_person_camerainfo_ = getCurrentCameraInfo();
    third_person_camerainfo_.is_orthographic = false;

    updateCamera(false);
}

void TerrainViewer::setCurrentCameraFromInfo(const CameraInfo &info)
{
    camera()->setPosition(castVec3f(info.position));
    camera()->setViewDirection(castVec3f(info.direction));
    camera()->setUpVector(castVec3f(info.up_vector));
    camera()->setFieldOfView(info.fov_in_rads);

    if (info.is_orthographic)
        camera()->setType(qglviewer::Camera::ORTHOGRAPHIC);
    else
        camera()->setType(qglviewer::Camera::PERSPECTIVE);

    //float zcoef = (info.zFar - camera()->distanceToSceneCenter())/camera()->sceneRadius();
    //camera()->setZClippingCoefficient(zcoef);

    ((MyCamera*)camera())->setZFar(info.zFar);
    ((MyCamera*)camera())->setZNear(info.zNear);
}

CameraInfo TerrainViewer::getCurrentCameraInfo() const
{
    CameraInfo info;
    info.position = castVec3f(camera()->position());
    info.direction = castVec3f(camera()->viewDirection());
    info.up_vector = castVec3f(camera()->upVector());
    info.screen_width = camera()->screenWidth();
    info.screen_height = camera()->screenHeight();
    info.fov_in_rads = camera()->fieldOfView();
    info.is_orthographic = (camera()->type() == qglviewer::Camera::ORTHOGRAPHIC);
    info.zNear = camera()->zNear();
    info.zFar = camera()->zFar();
    return info;
}

TerrainViewer::CameraMode TerrainViewer::getCameraMode() const
{
    return camera_mode_;
}

void TerrainViewer::setCameraMode(CameraMode camera_mode)
{
    if (camera_mode != camera_mode_)
    {
        camera_mode_ = camera_mode;
        if (camera_mode_ == kCameraFirstPersonMode)
            third_person_camerainfo_ = getCurrentCameraInfo();
        else if (camera_mode_ == kCameraThirdPersonMode)
            first_person_camerainfo_ = getCurrentCameraInfo();
        updateCamera();
    }
}

void TerrainViewer::updateCamera( bool update_display)
{
    if (camera_mode_ == kCameraFirstPersonMode)
        setCurrentCameraFromInfo(first_person_camerainfo_);
    else if (camera_mode_ == kCameraThirdPersonMode)
        setCurrentCameraFromInfo(third_person_camerainfo_);
    if (update_display) updateGL();
}

qglviewer::Vec TerrainViewer::getFirstPersonPosition()
{
    if (camera_mode_ == kCameraFirstPersonMode)
        return camera()->position();
    else
        return castVec3f(first_person_camerainfo_.position);
}

qglviewer::Vec TerrainViewer::getFirstPersonDirection()
{
    if (camera_mode_ == kCameraFirstPersonMode)
        return camera()->viewDirection();
    else
        return castVec3f(first_person_camerainfo_.direction);
}

void TerrainViewer::setFirstPersonPosition(qglviewer::Vec position)
{
    if (camera_mode_ == kCameraFirstPersonMode)
    {
        camera()->setPosition(position);
    }
    else
    {
        first_person_camerainfo_.position = castVec3f(position);
    }
}

float TerrainViewer::getAltitudeAtWorldCoords(float fx, float fy)
{
    return getTerrain()->getInterpolatedAltitude(fx,fy);
}

void TerrainViewer::adjustFirstPersonCamera()
{
    if (hasTerrain())
    {
        qglviewer::Vec position = getFirstPersonPosition();
        position.z = getAltitudeAtWorldCoords(position.x, position.y) + person_height_;
        setFirstPersonPosition(position);

        //        if (camera_mode_ == kCameraFirstPersonMode)
        //        {
        //            qglviewer::Vec direction = camera()->viewDirection();
        //            qglviewer::Vec further_position = camera()->position();

        //            float fx = (further_position.x - terrain_translation_.x)/terrain_scale_;
        //            float fy = (further_position.y - terrain_translation_.y)/terrain_scale_;

        //            int xmin = floor(fx);
        //            int ymin = floor(fy);
        //            if (xmin >= 0 && ymin >= 0 && xmin < terrain_->width() && ymin < terrain_->height())
        //            {
        //                int normal_index = (ymin*terrain_->width() + xmin)*3;
        //                qglviewer::Vec normal(normals_[normal_index],
        //                                      normals_[normal_index+1],
        //                                      normals_[normal_index+2]);
        //                qglviewer::Vec left(-direction.y, direction.x,  0.0);
        //                left.normalize();

        //                direction = left^normal;
        //                camera()->setViewDirection(direction);
        //            }
        //        }
    }
}

void TerrainViewer::keyPressEvent(QKeyEvent *event)
{
    switch (event->key())
    {
    case Qt::Key_Up:
        if (camera_mode_ == kCameraFirstPersonMode && !cur_first_person_camera_locked_)
        {
            qglviewer::Vec forward_direction = camera()->viewDirection();
            qglviewer::Vec prev_pos = camera()->position();
            prev_pos += forward_direction;

            camera()->setPosition(prev_pos);
            adjustFirstPersonCamera();
            updateGL();
            break;
        } else if (camera_mode_ == kCameraFirstPersonMode) break;

    case Qt::Key_Down:
        if (camera_mode_ == kCameraFirstPersonMode && !cur_first_person_camera_locked_)
        {
            qglviewer::Vec forward_direction = camera()->viewDirection();
            qglviewer::Vec prev_pos = camera()->position();
            prev_pos -= forward_direction;

            camera()->setPosition(prev_pos);
            adjustFirstPersonCamera();
            updateGL();
            break;
        } else if (camera_mode_ == kCameraFirstPersonMode) break;

    case Qt::Key_Left:
        if (camera_mode_ == kCameraFirstPersonMode && !cur_first_person_camera_locked_)
        {
            qglviewer::Quaternion rotateAroundUpAxis(qglviewer::Vec(0, 0, 1), kCameraRotateSteps);
            qglviewer::Vec direction = rotateAroundUpAxis.rotate(camera()->viewDirection());
            camera()->setViewDirection(direction);
            adjustFirstPersonCamera();
            updateGL();
            break;
        } else if (camera_mode_ == kCameraFirstPersonMode) break;

    case Qt::Key_Right:
        if (camera_mode_ == kCameraFirstPersonMode && !cur_first_person_camera_locked_)
        {
            qglviewer::Quaternion rotateAroundUpAxis(qglviewer::Vec(0, 0, 1), -kCameraRotateSteps);
            qglviewer::Vec direction = rotateAroundUpAxis.rotate(camera()->viewDirection());
            camera()->setViewDirection(direction);
            adjustFirstPersonCamera();
            updateGL();
            break;
        } else if (camera_mode_ == kCameraFirstPersonMode) break;

    case Qt::Key_Z:
        if (event->modifiers() & Qt::ControlModifier)
        {
            QString save_path  = QFileDialog::getSaveFileName(this,
                                                              "Save Camera Info",
                                                              QString(),
                                                              "CameraInfo (*.cam)" );
            if (save_path.size() > 0)
            {
                std::ofstream ofs(save_path.toStdString().c_str());
                getCurrentCameraInfo().write_text(ofs);
                ofs.close();
                break;
            }
        }

    case Qt::Key_T:
        if (event->modifiers() & Qt::ControlModifier && hasTerrain() && getCameraMode() == kCameraThirdPersonMode)
        {
            qglviewer::Vec position(first_person_camerainfo_.position[0],
                                    first_person_camerainfo_.position[1], width()/2.0f);
            qglviewer::Vec direction(0.0f,
                                     0.0f,
                                     - 1.0f);
            camera()->setPosition(position);
            camera()->setViewDirection(direction);
            //camera()->setUpVector(camera()->rightVector()^direction);
            updateGL();
            break;
        }

    default:
        Base::keyPressEvent(event);
        break;
    }
}

void TerrainViewer::wheelEvent(QWheelEvent *event)
{
    if (camera_mode_ == kCameraFirstPersonMode)
    {
        return;
    } else
        Base::wheelEvent(event);
}

void TerrainViewer::mouseMoveEvent(QMouseEvent *event)
{
    if (camera_mode_ == kCameraFirstPersonMode)
    {
        return;
    } else
        Base::mouseMoveEvent(event);
}

void TerrainViewer::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (camera_mode_ == kCameraThirdPersonMode && event->button() == Qt::LeftButton)
    {
        bool found;
        qglviewer::Vec new_position = camera()->pointUnderPixel(event->pos(), found);

        if (found)
        {
            setFirstPersonPosition(new_position);
            adjustFirstPersonCamera();
            updateGL();
        }
    }
}

QString TerrainViewer::helpString() const
{
	QString text("<h2>M y V i e w e r</h2>");
	text += "Note: You cannot rotate the terrain and zoom on/in the terrain when in first person mode. <br><br>";
	text += "Move the first person camera to a new position. Double click on the terrain. <br>";
	text += "See the whole terrain from the top when in third person mode. Press Ctrl + T. <br>";
	text += "Save current camera info. Press Ctrl + Z. <br>";
	text += "Move the first person camera when in first person mode. Use Left, Right, Up and Down keys. <br>";
	return text;
}

void TerrainViewer::draw()
{
  static float lightPos[4] = {0.5f, 1.0f, 0.0f, 0.0f}; //if w=0.0, light is directional
  static float lightSpecularCol[4] = {0.1f, 0.1f, 0.1f, 1.0f};
  static float lightDiffuseCol[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  static float lightAmbientCol[4] = {0.03f, 0.03f, 0.03f, 1.0f};

  glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecularCol);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuseCol);
  glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbientCol);
  glLightfv(GL_LIGHT0, GL_POSITION, lightPos);


    if (camera_mode_ == kCameraThirdPersonMode)
        drawFirstPerson();

    if (!hasTerrain())
    {
        loadFlatTerrain(2048, 2048);
        //loadTerrain("data/terrain_data/GC3.ter");
//        loadTerrain("result.ter");
        // loadFlatTerrain(512, 512);
    } else
    {
        glColor3f(0.5, 0.0, 0.0);
//        glBegin(GL_QUADS);
//        glVertex3f(0.0, 0.0, 0.0);
//        glVertex3f(getTerrain()->width(), 0.0, 0.0);
//        glVertex3f(getTerrain()->width(), getTerrain()->height(), 0.0);
//        glVertex3f(0.0, getTerrain()->height(), 0.0);
//        glEnd();

        drawTerrain();
        drawSilhouetteEdges(silhouette_segments_, Eigen::Vector3f(1.0, 1.0, 1.0));
    }
}

void TerrainViewer::drawFirstPerson()
{
    glColor3ub(0, 0, 0);
    glLineWidth(8.0);
    glDisable(GL_LIGHTING);

    float scale = person_height_;

    qglviewer::Vec first_person_pos = getFirstPersonPosition();
    glBegin(GL_LINES);
    glVertex3f(first_person_pos.x, first_person_pos.y, first_person_pos.z);
    glVertex3f(first_person_pos.x, first_person_pos.y, first_person_pos.z-person_height_);
    glEnd();

    qglviewer::Vec forward_direction = getFirstPersonDirection();
    forward_direction *= scale;
    drawArrow(first_person_pos, first_person_pos+forward_direction, 0.2);

    //    first_person_pos.z += scale;
    //    drawArrow(first_person_pos, first_person_pos+forward_direction, 0.2);

    glEnable(GL_LIGHTING);
    glLineWidth(1.0);
}

void TerrainViewer::drawSilhouetteEdges(std::vector<Terrain::SilhouetteSegment> &segments,
                                        Eigen::Vector3f edge_color)
{
    if (segments.size() == 0)
      return;

   glLineStipple(3, 0xAAAA);   //0x3F07

    glPushMatrix();

    glLineWidth(4.0);
    glDisable(GL_LIGHTING);
    for (unsigned int i = 0; i < segments.size(); ++i)
    {
        glColor3fv(edge_color.data());
        //Eigen::Vector3f color = heatcolor(i, 0, ((int)segments.size())-1);
        //std::cout << color[0] << " " << color[1] << " " << color[2] << std::endl << std::flush;

        //glColor3fv(heatcolor(i, 0, ((int)segments.size())-1).data());
        glBegin(GL_LINE_STRIP);
        for (unsigned int j = 0; j < segments[i].points.size(); ++j)
            glVertex3fv(segments[i].points[j].data());
        glEnd();

        glColor3f(0.8, 0.8, 0.0);
        glEnable(GL_LINE_STIPPLE);
        glBegin(GL_LINES);
        for (unsigned int j = 0; j < segments[i].points.size(); ++j)
        {
            if (j > 0 && j+1 < segments[i].points.size())
              if ((Eigen::Vector2f(segments[i].points[j][0], segments[i].points[j][1]) - Eigen::Vector2f(segments[i].points[j+1][0], segments[i].points[j+1][1])).norm() < 1)
                continue;
            glVertex3fv(segments[i].points[j].data());
            glVertex3f(segments[i].points[j][0], segments[i].points[j][1], getTerrain()->getInterpolatedAltitude(segments[i].points[j][0], segments[i].points[j][1]));
        }
        glEnd();
        glDisable(GL_LINE_STIPPLE);

        /*glColor3ub(255, 0, 0);
        glPointSize(5.0);
        glBegin(GL_POINTS);
        for (unsigned int j = 0; j < segments[i].points.size(); ++j)
            glVertex3fv(segments[i].points[j].data());
        glEnd();*/
    }
    glEnable(GL_LIGHTING);
    glLineWidth(1.0);

    glPopMatrix();
}

void TerrainViewer::drawTerrainGridNormals()
{
    glLineWidth(4.0);
    glColor3ub(0, 255, 0);
    glDisable(GL_LIGHTING);
    Terrain *terrain = getTerrain();

    glBegin(GL_LINES);
    for (int j = 0; j < terrain->height()-1; j+=5)
        for (int i = 0; i < terrain->width()-1; i+=5)
        {
            Terrain::Normal normal = terrain->getGridNormal(i, j);
            Eigen::Vector3f center = Eigen::Vector3f(i, j, terrain->getAltitude(i, j));
            center += Eigen::Vector3f(i+1, j+1, terrain->getAltitude(i+1, j+1));
            center += Eigen::Vector3f(i, j+1, terrain->getAltitude(i, j+1));
            center += Eigen::Vector3f(i+1, j, terrain->getAltitude(i+1, j));
            center /= 4.0;
            Eigen::Vector3f displaced = (center + normal);

            glColor3ub(0, 255, 0);
            glVertex3fv(center.data());
            glVertex3fv(displaced.data());

//            Eigen::Vector3f vec1, vec2, normal2;
//            vec1 = Eigen::Vector3f(i+1, j+1, terrain->getAltitude(i+1, j+1)) - Eigen::Vector3f(i, j, terrain->getAltitude(i, j));
//            vec2 = Eigen::Vector3f(i, j+1, terrain->getAltitude(i, j+1)) - Eigen::Vector3f(i+1, j, terrain->getAltitude(i+1, j));
//            normal2 = vec1.cross(vec2).normalized();
//            Eigen::Vector3f displaced2 = (center + normal2);


//            glColor3ub(255, 0, 0);
//            glVertex3fv(center.data());
//            glVertex3fv(displaced2.data());
        }
    glEnd();

    glEnable(GL_LIGHTING);
    glLineWidth(1.0);

}

void TerrainViewer::drawTerrain()
{
    if (wireframe_)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    terrain_renderer_.display();
}

void TerrainViewer::extractSilhouetteEdges()
{
  if (getCameraMode() == kCameraFirstPersonMode)
      first_person_camerainfo_ = getCurrentCameraInfo();
  extractSilhouetteEdges(first_person_camerainfo_, silhouette_segments_);
  updateGL();
}

void TerrainViewer::extractSilhouetteEdges(const CameraInfo &camera_info, std::vector<Terrain::SilhouetteSegment> &result)
{
  result.clear();
  if (hasTerrain())
  {
      if (getCameraMode() == kCameraFirstPersonMode)
          first_person_camerainfo_ = getCurrentCameraInfo();
      SilhouetteExtractor extractor(getTerrain());
      extractor.compute(camera_info , result);
      gotSilhouettes_ = true;

  }
}

void TerrainViewer::extractRidges()
{
  if (getCameraMode() == kCameraFirstPersonMode)
      first_person_camerainfo_ = getCurrentCameraInfo();
  extractRidges(first_person_camerainfo_, silhouette_segments_);
  updateGL();
}

void TerrainViewer::extractRidges(const CameraInfo &camera_info, std::vector<Terrain::SilhouetteSegment> &result)
{
  result.clear();
  if (hasTerrain())
  {
      FeatureDetection ridges_extractor(getTerrain());
      ridges_extractor.computeRidges(first_person_camerainfo_, result);
      gotSilhouettes_ = false;
  }
}

void TerrainViewer::extractFeatures()
{
  if (getCameraMode() == kCameraFirstPersonMode)
      first_person_camerainfo_ = getCurrentCameraInfo();
  extractFeatures(first_person_camerainfo_, silhouette_segments_);
  updateGL();
}

void TerrainViewer::extractFeatures(const CameraInfo &camera_info, std::vector<Terrain::SilhouetteSegment> &result)
{
  result.clear();
  if (hasTerrain())
  {
      extractRidges(camera_info, result);

      std::vector<Terrain::SilhouetteSegment> segs;
      extractSilhouetteEdges(camera_info, segs);
      result.insert(result.begin(), segs.begin(), segs.end());
  }
}

void TerrainViewer::setPersonHeight(float height)
{
    person_height_ = height;
    adjustFirstPersonCamera();
    updateGL();
}

void TerrainViewer::setTerrain(Terrain *terrain, bool update_display)
{
    delete terrain_renderer_.getTerrain();
    terrain_renderer_.setTerrain(terrain);
    if (update_display) updateTerrain();
}

void TerrainViewer::updateTerrain(bool update_person)
{
    emit sendStatusMessage("Updating terrain ...");
    terrain_renderer_.updateTerrain();

    if (hasTerrain())
    {
        float scale = getTerrain()->xscale();
        float min_alt = getTerrain()->min_altitude();
        int terrain_width = getTerrain()->width();
        int terrain_height = getTerrain()->height();

//        ((MyCamera*)camera())->setZFar(terrain_width);
//        first_person_camerainfo_.zFar = terrain_width;
//        third_person_camerainfo_.zFar = terrain_width;

        //setFirstPersonPosition(qglviewer::Vec(scale*terrain_->width()/2.0,
        //                                      scale*terrain_->height()/2.0, 0.0));
        if (update_person) adjustFirstPersonCamera();
        
//        camera()->setFieldOfView(M_PI*1.1);

        if (camera_mode_ == kCameraFirstPersonMode)
            first_person_camerainfo_ = getCurrentCameraInfo();

        setCurrentCameraFromInfo(third_person_camerainfo_);
        camera()->setSceneCenter(qglviewer::Vec(scale*terrain_width/2.0,
                                                scale*terrain_height/2.0,
                                                scale*min_alt));
#if (QGLVIEWER_VERSION >= 0x020500)
        camera()->setPivotPoint(camera()->sceneCenter());
#else
        camera()->setRevolveAroundPoint(camera()->sceneCenter());
#endif
        camera()->setSceneRadius(scale*terrain_width/2.0);
        camera()->showEntireScene();
        third_person_camerainfo_ = getCurrentCameraInfo();

        updateCamera();
    }

    updateGL();
}

void TerrainViewer::loadFlatTerrain(int terrain_width, int terrain_height)
{
    setTerrain(new Terrain (terrain_width, terrain_height));

    //setFirstPersonPosition(qglviewer::Vec(getTerrain()->width()/2.0, 0.0, 0.0));
    //first_person_camerainfo_.direction = Eigen::Vector3f(0.0, 1.0, 0.0);
    //adjustFirstPersonCamera();
}

bool TerrainViewer::loadTerrain(const char *terrain_path)
{
    setTerrain(NULL, false);
    Terrain *terrain = new Terrain();

    if (terrain->load(terrain_path))
    {
        setTerrain(terrain);
        return true;
    }
    else
    {
        std::cerr << "Could not open terrain at " << terrain_path << "\n" << std::flush;
        delete terrain;
        return false;
    }
}

bool TerrainViewer::saveTerrain(const char *terrain_path)
{
    if (!getTerrain()->save(terrain_path))
    {
        std::cerr << "Could not save terrain at " << terrain_path << "\n" << std::flush;
        return false;
    } else
        return true;
}

void TerrainViewer::on_cameramodeCheckBox_toggled(bool status)
{
    if (status)
        setCameraMode(kCameraFirstPersonMode);
    else
        setCameraMode(kCameraThirdPersonMode);
}
