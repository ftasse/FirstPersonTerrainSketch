#include "sketch_viewer.h"

#include <float.h>
#include <fstream>
#include <string>

#include <GL/glu.h>
#include <QMouseEvent>
#include <QPoint>
#include <QFileInfo>
#include <QImage>
#include <QtConcurrentRun>
#include <QThread>

#include "tools/offline_terrain_renderer.h"
#include "algorithms/deformation/deformation.h"
#include "algorithms/deformation/feature_matching.h"
#include "algorithms/deformation/extended_feature_matching.h"
#include "algorithms/deformation/protruded_silhouettes_matching.h"
#include "algorithms/conversion_to_3d.h"
#include "algorithms/polyline_completion.h"

#define kLoadTerrainWithPickedSketch 1

void nurbsError(GLenum errorCode);

void LowerSilhouettesWorker::doWork()
{
  std::vector<CameraInfo> camera_infos;
  for (unsigned int i = 0; i < sketches_.size(); ++i)
    camera_infos.push_back(sketches_[i]->camera_info());

  for (unsigned int i = 0; i < sketches_.size(); ++i)
  {
      Sketch* sketch_ = sketches_[i];
      ProtrudedSilhouetteMatching matching(terrain_,sketch_);

      double deformation_error = FLT_MAX;
      double old_deformation_error = 0;
      int niter = 1;

          double diff_eps = 2e0; // FLT_EPSILON

      while (deformation_error > 1e-1 && std::abs(old_deformation_error-deformation_error) > diff_eps)
      {
          fprintf(stdout, "Iteration %d: \n", niter);
          fflush(stdout);
          if (niter > 1) old_deformation_error= deformation_error;

          matching.setDeformedFeatures(constraint_edges_);
          matching.updateSilhouettes();

          constraint_edges_ = matching.getDeformedFeatures();

          deformation_error = deformTerrainWithConstraints(terrain_, camera_infos, constraint_edges_);
//          CameraInfo camera_info = sketch_->camera_info();
//          camera_info.position[2] = terrain_->getInterpolatedAltitude(sketch_->camera_info().position[0], sketch_->camera_info().position[1]);
//          sketch_->setCameraInfo(camera_info);
          emit sendUpdateTerrain(false);
          ++niter;

          fprintf(stdout, "diff error: %lf, %lf, %lf \n", deformation_error, old_deformation_error,  std::abs(old_deformation_error-deformation_error));

          // if (matching.numUnallocatedSilhouettes() == 0) break;

      }

      fprintf(stdout, "Lower silhouettes done for sketch %d! \n\n", i);
      fflush(stdout);
    }

    fprintf(stdout, "Lower silhouettes done! \n");
    fflush(stdout);
}

SketchViewer::SketchViewer(QWidget *parent):
    Base(parent), is_sketching_(false),
    sketch_mode_(kSketchIdleMode),
    cur_sketch_index_(-1),
    cur_stroke_index_(-1),
    tree_widget_(NULL)
{
}

SketchViewer::~SketchViewer()
{
    for (unsigned int i = 0; i < sketches_.size(); ++i)
        delete sketches_[i];
}

void SketchViewer::lowerSilhouettes()
{
    if (hasTerrain() && hasSketches())
    {
            LowerSilhouettesWorker *worker = new LowerSilhouettesWorker(getTerrain(), sketches_, constraint_edges_);
            QObject::connect(worker, SIGNAL(sendUpdateTerrain(bool)), this, SLOT(updateTerrain(bool)), Qt::QueuedConnection);

            QThread *thread = new QThread;
            worker->moveToThread(thread);
            thread->start();
            QMetaObject::invokeMethod(worker, "doWork", Qt::QueuedConnection);
    }
}

void SketchViewer::deformTerrain()
{
    if (hasTerrain() && hasSketches())
    {
        if (constraint_edges_.size() == 0) matchStrokesToFeatures();

        if (constraint_edges_.size() == 0) {
            for (unsigned int i = 0; i < sketches_.size(); ++i)
            {
                Deformation *deformation = new Deformation(getTerrain(), sketches_[i]);
                deformation->update();
                delete deformation;
            }
        }
        else
        {
          // embedSketch();
          /*std::vector<Terrain::SilhouetteSegment> segs;
          FeatureDeformation deformation(getTerrain(), currentSketch(), segs);
          deformation.setDeformedFeatures(constraint_edges_);
          deformation.deform();
          constraint_edges_ = deformation.getDeformedFeatures(); // protuded edges ... ?*/

            std::vector<CameraInfo> camera_infos;
            getSketchCameraInfos(camera_infos);
            deformTerrainWithConstraints(getTerrain(), camera_infos, constraint_edges_);

        }
        updateTerrain(false);
    }
}

void SketchViewer::matchStrokesToFeatures()
{
  if (hasTerrain() && hasSketches())
  {
      if (silhouette_segments_.size() > 0)
      {
          ExtendedFeatureMatching matching(getTerrain(), sketches_, terrain_features_per_sketch_);
          matching.update(weighting_);
          constraint_edges_ = matching.getDeformedFeatures();
      } else
       {
          constraint_edges_.clear();
          for (unsigned int i = 0; i < sketches_.size(); ++i)
            {
              Sketch *sketch = sketches_[i];
              for (unsigned int j = 0; j < sketch->numStrokes(); ++j)
                {
                  Stroke *stroke = sketch->getStroke(j);
                  Eigen::Vector3f point_on_terrain = stroke->getControlPoint(0);

                  Eigen::Vector3f ray = (stroke->getControlPoint(0) - sketch->camera_info().position).normalized();
                  std::vector<Eigen::Vector3f> intersects;
                  bool intersectTerrain = rayTerrainIntersect(getTerrain(), sketch->camera_info().position, ray, getTerrain()->width(), intersects, 1e0);
                  if (intersectTerrain) point_on_terrain = intersects.front();

                  Terrain::SilhouetteSegment seg;

                  for (unsigned int k = 0; k < stroke->numControlPoints(); ++k)
                    {
                      Eigen::Vector3f point = stroke->getControlPoint(k);
                      Eigen::Vector3f ray_dir = (point - sketch->camera_info().position).normalized();

                      Eigen::Vector3f projection = projectOnPlane(point_on_terrain, -sketch->camera_info().direction, sketch->camera_info().position, ray_dir);
                      seg.points.push_back(projection);
                    }
                  constraint_edges_.push_back(seg);
                }
            }
        }

      updateSketches();
  }
}

void SketchViewer::extractSilhouetteEdges()
{
  terrain_features_per_sketch_.clear();

  if (sketches_.size() == 0)  Base::extractSilhouetteEdges();
  else
  {
      terrain_features_per_sketch_.resize(sketches_.size());
      for (unsigned int i = 0; i < sketches_.size(); ++i)
        {
          Base::extractSilhouetteEdges(sketches_[i]->camera_info(), terrain_features_per_sketch_[i]);
        }
      if (cur_sketch_index_ >= 0) silhouette_segments_ = terrain_features_per_sketch_[cur_sketch_index_];
      else silhouette_segments_ = terrain_features_per_sketch_[0];
      updateGL();
  }
}

void SketchViewer::extractRidges()
{
  terrain_features_per_sketch_.clear();
  Base::extractRidges();

  if (sketches_.size() > 0)
  {
      terrain_features_per_sketch_.resize(sketches_.size(), silhouette_segments_);
  }
}

void SketchViewer::extractFeatures()
{
  extractRidges();

  if (sketches_.size() > 0)
  {
      for (unsigned int i = 0; i < sketches_.size(); ++i)
      {
          std::vector<Terrain::SilhouetteSegment> segs;
          Base::extractSilhouetteEdges(sketches_[i]->camera_info(), segs);
          terrain_features_per_sketch_[i].insert(terrain_features_per_sketch_[i].end(), segs.begin(), segs.end());
      }

      if (cur_sketch_index_ >= 0) silhouette_segments_ = terrain_features_per_sketch_[cur_sketch_index_];
      else silhouette_segments_ = terrain_features_per_sketch_[0];
      updateGL();
  }
}

void SketchViewer::deleteStroke(int sketch_index, int stroke_index)
{
    Sketch *sketch = sketchAt(sketch_index);
    if (sketch)
    {
        Stroke *stroke = sketch->getStroke(stroke_index);
        if (stroke)
        {
            sketch->deleteStroke(stroke_index);
            if (sketch->numStrokes() > 0)
            {
              ConversionTo3D conversion_to_3d(sketches_[sketch_index]);
              conversion_to_3d.sortStrokes();
            }
            setCurrentStrokeIndex(-1);
            updateSketches();
        }
    }
}

void SketchViewer::embedSketches()
{
    for (unsigned int i = 0; i < sketches_.size(); ++i)
      if (sketches_[i]->numStrokes() > 0)
    {
        //Place all strokes all the same plane. remove this at a later stage
        makeSketchPlanar(sketches_[i]);

        ConversionTo3D conversion_to_3d(sketches_[i]);
        //conversion_to_3d.convert();
        conversion_to_3d.sortStrokes();
        updateSketches();
    }
}

void SketchViewer::makeSketchPlanar(Sketch* sketch)
{
  if (sketch != NULL)
  {
      Eigen::Vector3f plane_normal = -sketch->camera_info().direction;
      Eigen::Vector3f point_on_plane = sketch->getStroke(0)->getControlPoint(0);
      for (int i = 1; i < sketch->numStrokes(); ++i)
        {
          Stroke *stroke = sketch->getStroke(i);
          for (int j = 0; j < stroke->numControlPoints(); ++j)
            {
              Stroke::Point point = stroke->getControlPoint(j);
              Stroke::Point projected = projectOnPlane(point_on_plane, plane_normal,
                                                       sketch->camera_info().position,
                                                       (point - sketch->camera_info().position).normalized());
              stroke->setControlPoint(j, projected);
            }
        }
  }
}

void SketchViewer::keyPressEvent(QKeyEvent *event)
{
    switch(event->key())
    {
    case Qt::Key_R:
        reloadSketches();
        break;
    case Qt::Key_A:
      if (event->modifiers() & Qt::ControlModifier)
      {
          silhouette_segments_.clear();
          picked_sihouette_points_.clear();
          picked_sihouette_colors_.clear();

          constraint_edges_.clear();
          constraint_edges_on_terrain_.clear();
          updateGL();
          break;
      }
    case Qt::Key_D:
      if (event->modifiers() & Qt::ControlModifier)
      {
          silhouette_segments_.clear();
          picked_sihouette_points_.clear();
          picked_sihouette_colors_.clear();
          updateGL();
          break;
      }
      case Qt::Key_P:
        if (event->modifiers() & Qt::ControlModifier)
        {
            for (unsigned int i = 0; i < sketches_.size(); ++i) makeSketchPlanar(sketches_[i]);
            updateSketches();
            break;
        }
      case Qt::Key_C:
        if (event->modifiers() & Qt::ControlModifier)
        {
            //sketch_->completeEnds();

            std::vector< std::vector<Eigen::Vector3f> > poly_points (constraint_edges_.size());

            for (unsigned int i = 0; i < constraint_edges_.size(); ++i)
              poly_points[i] = constraint_edges_[i].points;

            PolylineCompletion completion;
            completion.setPolylines(poly_points);
            completion.completePolylines(getTerrain());
            completion.getCompletedPolylines(poly_points);

            for (unsigned int i = 0; i < constraint_edges_.size(); ++i)
                constraint_edges_[i].points = (poly_points[i]);

            updateSketches();
            break;
        }
    default:
        Base::keyPressEvent(event);
        break;
    }
}

void SketchViewer::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && sketch_mode_ == kSketchDrawMode)
    {
        startSketching(event->pos());
    } else
    {
        Base::mousePressEvent(event);
    }
}

void SketchViewer::mouseReleaseEvent(QMouseEvent *event)
{
    if (sketch_mode_ == kSketchDrawMode && is_sketching_)
        stopSketching();
    else
    {
        Base::mouseReleaseEvent(event);
    }
}

void SketchViewer::mouseMoveEvent(QMouseEvent *event)
{
    if (sketch_mode_ == kSketchDrawMode && is_sketching_ && currentSketch()->getActivePolyline() != NULL)
    {
        Stroke::Point point;
        if (pixelToWorldCoords(event->pos(), point))
            addToSketch(point);
    }
    else
    {
        Base::mouseMoveEvent(event);
    }
}

QString SketchViewer::helpString() const
{
	QString text = Base::helpString() + "\n";
	text += "Note: When in sketch mode, after a click to set the start point of the stroke, move the mouse to draw a stroke. <br>";
	text += "Note: To deform a terrain, first press <b>Detect Ridges</b> to detect both ridges and silhouettes.";
	text += "Then press <b>Deform Terrain</b> once to run the energy minimization and once again to actually deform the terrain. <br><br>";

	text += "Start drawing a stroke in sketch mode. Mouse click at the starting position of the stroke and drag.<br>";
	text += "Stop drawing a stroke. Mouse release at the end position of the stroke. <br>";
	text += "Complete the strokes of a sketch. Press Ctrl + C. <br>";
	text += "Make the sketch planar. Press Ctrl + P. <br>";
	text += "Delete feature edges and assigned features on the terrain surface. Press Ctrl + D. <br>";
	text += "Delete all strokes and edges except for the actual sketch. Press Ctrl + A. <br>";
	return text;
}

void SketchViewer::init()
{
    Base::init();
    glEnable(GL_MAP1_VERTEX_3);
}

void SketchViewer::draw()
{
    Base::draw();

    glPushMatrix();
    glDisable(GL_LIGHTING);
    glLineWidth(3.0);
    for (unsigned int i = 0; i < sketches_.size(); ++i)
        drawSketch(sketches_[i]);

    drawSilhouetteEdges(constraint_edges_, Eigen::Vector3f(0.0, 1.0, 0.0));
    /*for (unsigned int i = 0; i < constraint_edges_.size(); ++i)
    {
        glColor3fv(heatcolor(i/(((float)constraint_edges_.size())-1)).data());
        glBegin(GL_LINE_STRIP);
        for (unsigned int j = 0; j < constraint_edges_[i].points.size(); ++j)
          glVertex3fv(constraint_edges_[i].points[j].data());
        glEnd();
      }*/

    /*glBegin(GL_QUADS);
    for (unsigned int i=0; i< constraint_edges_.size(); ++i)
    {
        Eigen::Vector3f left_pt = constraint_edges_[i].points.front();
        Eigen::Vector3f right_pt = constraint_edges_[i].points.back();
        float terrain_diagonal_length = sqrt(512*512 + 512*512);
        Eigen::Vector3f eye = first_person_camerainfo_.position;
        Eigen::Vector3f v = right_pt + terrain_diagonal_length*(right_pt-eye).normalized();
        Eigen::Vector3f w = left_pt + terrain_diagonal_length*(left_pt-eye).normalized();
        left_pt[2] = right_pt[2] = v[2] = w[2] = 100;
        glVertex3fv(left_pt.data());
        glVertex3fv(right_pt.data());
        glVertex3fv(v.data());
        glVertex3fv(w.data());
    }
    glEnd();*/

    // if (getCameraMode() == kCameraThirdPersonMode)
    //   drawSilhouetteEdges(constraint_edges_on_terrain_, Eigen::Vector3f(0.0, 0.0, 0.0));

    glColor3f(1, 0, 0);
    glPointSize(5.0);
    glBegin(GL_POINTS);
    for (unsigned int j = 0; j < picked_sihouette_points_.size(); j++)
    {
      glColor3fv(picked_sihouette_colors_[j].data());
      glVertex3fv(picked_sihouette_points_[j].data());
    }
    glEnd();
    glPointSize(1.0);

    glLineWidth(1.0);
    glEnable(GL_LIGHTING);
    glPopMatrix();

    if (hasTerrain())
    {
        float sky_height = 1000.0f;
        float w = getTerrain()->width()-1;
        float h = getTerrain()->height()-1;

        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
       /* glBegin(GL_QUADS);
        glVertex3f(0.0, 0.0, sky_height);
        glVertex3f(0.0, 0.0, -sky_height);
        glVertex3f(w, 0.0, -sky_height);
        glVertex3f(w, 0.0, sky_height);

        glVertex3f(w, 0.0, sky_height);
        glVertex3f(w, 0.0, -sky_height);
        glVertex3f(w, h, -sky_height);
        glVertex3f(w, h, sky_height);

        glVertex3f(w, h, sky_height);
        glVertex3f(w, h, -sky_height);
        glVertex3f(0.0, h, -sky_height);
        glVertex3f(0.0, h, sky_height);

        glVertex3f(0.0, h, sky_height);
        glVertex3f(0.0, h, -sky_height);
        glVertex3f(0.0, 0.0, -sky_height);
        glVertex3f(0.0, 0.0, sky_height);
        glEnd();*/

        GLUquadric *quadric = gluNewQuadric();
        gluQuadricDrawStyle(quadric, GLU_FILL);
        gluSphere(quadric, sky_height/2, 20, 20);
        gluDeleteQuadric(quadric);

        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    }
}

void SketchViewer::drawSketch(Sketch* sketch)
{
    if (sketch != NULL)
    {
        for (int i = 0; i < sketch->numStrokes(); ++i)
        {
            float factor = i/((float) sketch->numStrokes());
            //glColor3f(0, 0, 0);
            glColor3fv(heatcolor(i, 0, sketch->numStrokes()-1).data());

            Stroke *stroke = sketch->getStroke(i);
            drawStroke(sketch, stroke);
        }
        glColor3ub(255, 0, 0);
        drawPolyline(sketch->getActivePolyline());
    }
}

void SketchViewer::drawStroke(Sketch *sketch, Stroke *stroke)
{
    if (stroke != NULL)
    {
        if (currentSketch() == sketch && cur_stroke_index_ >= 0 && sketch->getStroke(cur_stroke_index_) == stroke)
        {
            glLineStipple(1, 0xAAAA);   //0x3F07
            glEnable(GL_LINE_STIPPLE);
        }

        Eigen::Vector3f eye = sketch->camera_info().position;
        Eigen::Vector3f dir = sketch->camera_info().direction;

        std::vector<float> control_points(stroke->numControlPoints()*3, 0.0f);
        for (int j = 0; j < stroke->numControlPoints(); ++j)
        {   Eigen::Vector3f ray_dir = (stroke->getControlPoint(j) - eye).normalized();
            float dist = (eye - stroke->getControlPoint(j)).norm();
            //Eigen::Vector3f projected = projectOnPlane((eye + ray_dir*dist) + j*dir.normalized(),-dir, eye, ray_dir);
            Eigen::Vector3f projected = projectOnPlane((eye + ray_dir*10),-dir, eye, ray_dir);

            for (int k = 0;  k < 3; ++k)
                control_points[j*3 + k] = projected[k];
        }
        //if  (stroke->closed()) glVertex3fv(stroke->getControlPoint(0).data());

        // Draw curve
//        nurbs_ = gluNewNurbsRenderer();
//        gluNurbsCallback(nurbs_, GLU_ERROR, (GLvoid (*)()) nurbsError);
//        gluBeginCurve(nurbs_);
//        gluNurbsCurve(nurbs_,
//                      stroke->numKnots(),
//                      stroke->knotsPtr(),
//                      3,
//                      &control_points[0],
//                      stroke->order(),
//                      GL_MAP1_VERTEX_3);
//        gluEndCurve(nurbs_);
//        gluDeleteNurbsRenderer(nurbs_);
        glBegin(GL_LINE_STRIP);
        for (int j = 0; j < stroke->numControlPoints()-1; j++)
        {
            glVertex3fv(&control_points[j*3]);
            glVertex3fv(&control_points[(j+1)*3]);
        }
        glEnd();

        // Draw control points
        /*glColor3f(0, 1, 1);
        glPointSize(3.0);
        glBegin(GL_POINTS);
        for (int j = 0; j < stroke->numControlPoints(); j++)
            glVertex3fv(&control_points[j*3]);
        glEnd();
        glPointSize(1.0);*/


//        if (getCameraMode() == kCameraThirdPersonMode)
//        {
//            glLineStipple(1, 0xAAAA);
//            glEnable(GL_LINE_STIPPLE);
//            glBegin(GL_LINES);
//            for (int j = 0; j < stroke->numControlPoints(); j++)
//            {
//                glVertex3fv(stroke->getControlPoint(j).data());

//                Stroke::Point point = stroke->getControlPoint(j);
//                point = projectOnPlane(Eigen::Vector3f(0.0, 0.0, 0.0),
//                                       Eigen::Vector3f(0.0, 0.0, 1.0),
//                                       point);
//                if (hasTerrain())
//                    point[2] = getTerrain()->getInterpolatedAltitude(point[0], point[1]);
//                else point[2]  = 0.0f;
//                glVertex3fv(point.data());
//            }
//            glEnd();
//            glDisable(GL_LINE_STIPPLE);
//        }

        glDisable(GL_LINE_STIPPLE);
    }
}

void SketchViewer::drawPolyline(TerrSketch::Polyline *polyline)
{
    if (polyline != NULL)
    {
        glColor3f(0, 0, 1);
        glBegin(GL_LINE_STRIP);
        for (int j = 0; j < polyline->numPoints(); ++j)
        {
            //TODO remove the rescaling
            Stroke::Point point = polyline->getPoint(j);
            glVertex3fv(point.data());
        }
        if  (polyline->closed())
            glVertex3fv(polyline->getPoint(0).data());
        glEnd();

        glColor3f(1, 0, 1);
        glPointSize(3.0);
        glBegin(GL_POINTS);
        for (int j = 0; j < polyline->numPoints(); ++j)
        {
            //TODO remove the rescaling
            Stroke::Point point = polyline->getPoint(j);
            glVertex3fv(point.data());
        }
        glPointSize(1.0);
        glEnd();
    }
}

void SketchViewer::updateSketches()
{
  /*constraint_edges_on_terrain_.clear();
  if (constraint_edges_.size() > 0)
    {
      for (unsigned int i = 0; i < constraint_edges_.size(); ++i)
        {
          Terrain::SilhouetteSegment seg = constraint_edges_[i];
          for (unsigned int j = 0; j < seg.points.size(); ++j)
            seg.points[j][2] = getTerrain()->getInterpolatedAltitude(seg.points[j][0], seg.points[j][1]);
          constraint_edges_on_terrain_.push_back(seg);
        }
    }*/
    updateGL();
}

void SketchViewer::updateTerrain(bool update_person)
{

  if (hasSketches())
    {
//      Eigen::Vector3f pos = sketch_->camera_info().position;
//      adjustFirstPersonCamera();
//      float hdiff = first_person_camerainfo_.position[2] - pos[2];
//      std::cout << "Val: " << hdiff << std::endl << std::flush;
//      if (std::abs(hdiff) > FLT_EPSILON)
//        {
//          sketch_->camera_info().position = first_person_camerainfo_.position;
//          for (int i = 0; i < sketch_->numStrokes(); ++i)
//            sketch_->getStroke(i)->displaceHeight(hdiff);
//        }
    }
  TerrainViewer::updateTerrain(update_person);

}

void SketchViewer::switchFirstPersonCamera(Sketch* sketch)
{
    // if (getCameraMode() == kCameraFirstPersonMode)
    {
        if (sketch != NULL)
        {
            if (!cur_first_person_camera_locked_) 
            {
                if (getCameraMode() == kCameraFirstPersonMode)
                    first_person_camerainfo_ = getCurrentCameraInfo();
                saved_first_person_camerainfo_ = first_person_camerainfo_;
            }
            cur_first_person_camera_locked_ = true;
            first_person_camerainfo_ = sketch->camera_info();
            std::cout << "Lock first person camera" <<  " "  << saved_first_person_camerainfo_.position << "\n";
        } else
        {
            first_person_camerainfo_ = saved_first_person_camerainfo_;
            cur_first_person_camera_locked_ = false;
            std::cout << "Unlock first person camera" <<  " "  << first_person_camerainfo_.position << "\n";
        }
        updateCamera();
    }
}

bool SketchViewer::loadSketch(const char *sketch_path)
{
    Sketch *sketch = new Sketch();
    bool success = false;

    if (QFileInfo(sketch_path).suffix().compare("svg") == 0)
    {
        success = sketch->loadSvg(sketch_path);
        if (hasTerrain())
            sketch->setDefaultCamera(Eigen::Vector2i(getTerrain()->width(),
                                                     getTerrain()->height()));
        else
            sketch->setDefaultCamera();
    } else
        success = sketch->load(sketch_path);

    if (success)
    {
        addSketch(sketch);

        if (sketch !=  NULL && sketch->numStrokes() > 0)
        {
            switchFirstPersonCamera(sketch);
            setCurrentSketchIndex(sketches_.size() - 1);
        }
        for (int m=0; m < sketch->numStrokes(); ++m)
        {
            sketch->getStroke(m)->simplify(0.0, 0.5);
        }
        updateSketches();
        return true;
    } else
    {
        std::cerr << "Could not open sketch at " << sketch_path << "\n" << std::flush;
        delete sketch;
        return false;
    }
}

bool SketchViewer::saveSketch(const char *sketch_path, Sketch* sketch)
{
    if (sketch)
    {
        if (!sketch->save(sketch_path))
        {
            std::cerr << "Could not save sketch at " << sketch_path << "\n" << std::flush;
            return false;
        } else
            return true;
    } else
    {
        bool success = true;
        for (unsigned int i = 0; i < sketches_.size(); ++i)
        {
            std::stringstream ss;
            std::string path = sketch_path;
            size_t dot = path.find(".");
            std::string fname = path, extension = "";
            if (dot != std::string::npos)
            {
                fname = path.substr(0, dot);
                extension = path.substr(dot, path.size() - dot);
            }
            ss << fname << "_" << i+1 << extension;
            if (!saveSketch(ss.str().c_str(), sketches_[i]))    success = false;
        }
        return success;
    }
}

void SketchViewer::startSketching(QPoint start_pos)
{
    assert(sketch_mode_ == kSketchDrawMode);

    std::cout << "Start sketching: " << cur_stroke_index_ << "\n";

    glGetDoublev(GL_MODELVIEW_MATRIX, modeview_matrix_);
    glGetDoublev(GL_PROJECTION_MATRIX, projection_matrix_);
    glGetIntegerv(GL_VIEWPORT, viewport_);

    Stroke::Point start_point;
    if (pixelToWorldCoords(start_pos, start_point))
    {
        if (currentSketch() == NULL)   startNewSketch();
        currentSketch()->startStroke();
        is_sketching_ = true;

        qglviewer::Vec view_direction = camera()->viewDirection();
        drawing_plane_.point_on_plane = start_point;
        drawing_plane_.normal = Stroke::Point(view_direction.x,
                                              view_direction.y,
                                              view_direction.z);

        addToSketch(start_point);
    }
}

void SketchViewer::stopSketching()
{
    assert(sketch_mode_ == kSketchDrawMode);
    is_sketching_ = false;

    Sketch *sketch = currentSketch();

    sketch->getActivePolyline()->simplify(0.0, 0.5);

    if (sketch->getActivePolyline()->numEdges() > 2)
    {
//        sketch_->getActivePolyline()->simplify(1.0, 1.0);
//        sketch_->getActivePolyline()->simplifyDouglasPeucker(0.5);
//        sketch_->getActivePolyline()->simplifyDouglasPeucker(0.2);
        sketch->setCameraInfo(getCurrentCameraInfo());
        sketch->stopStroke();
        addStroke(cur_sketch_index_, sketch->getCurrentStroke());
        setCurrentStrokeIndex(sketch->getCurrentStrokeId());
        sketch->getCurrentStroke()->adaptForTerrain(sketch->camera_info());
        embedSketches();
        //updateSketches();
    }
}

void SketchViewer::addToSketch(const Stroke::Point world_coords)
{
    Sketch *sketch = currentSketch();

    assert(sketch_mode_ == kSketchDrawMode && sketch->getActivePolyline() != NULL);

    Stroke::Point projected;
    bool success = false;
    bool is_orthographic = camera()->type() == qglviewer::Camera::ORTHOGRAPHIC;
    success = projectOnDrawingPlane(world_coords, projected, is_orthographic);
    if (success && getTerrain()->validCoordinates(projected[0], projected[1]))
    {
        sketch->addPolylinePoint(projected);
        updateGL();
    }
}

bool SketchViewer::pixelToWorldCoords(const QPoint mousePos,
                                      Stroke::Point &world_coords)
{
    float srcZ = 1.0;
    double objX, objY, objZ;

    glReadPixels(mousePos.x(), height()-mousePos.y(), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &srcZ);
    if (srcZ >= 1.0f) srcZ = 0.9991f;

    bool succeded = gluUnProject(mousePos.x(), height()-mousePos.y(), srcZ,
                                 modeview_matrix_, projection_matrix_, viewport_,
                                 &objX, &objY, &objZ);

    if (!succeded)
    {
        std::cerr << "Warning: could not unproject screen point at: "
                  << mousePos.x() << " " << mousePos.y() << std::endl << std::flush;
        return false;
    } else
    {
        world_coords = Stroke::Point(objX, objY, objZ);
        return true;
    }
}

bool SketchViewer::projectOnDrawingPlane(const Stroke::Point point,
                                         Stroke::Point &projected,
                                         bool orthographic_camera)
{
//    projected = point;
//    return true;

    // this comes down to intersection betwen drawing plane and projection line
    // in perspective camera case, projection line is line from camera eye to point
    // http://geomalgorithms.com/a05-_intersect-1.html

    if (!orthographic_camera)
    {
        Stroke::Point point2 = castVec3f(camera()->position());
        Stroke::Point u  = (point - point2).normalized();
        projected = projectOnPlane(drawing_plane_.point_on_plane, drawing_plane_.normal, point2, u);
        return true;
    } else
    {
        float t = drawing_plane_.normal.dot(drawing_plane_.point_on_plane);
        t -= drawing_plane_.normal.dot(point);
        t /= drawing_plane_.normal.dot(drawing_plane_.normal);

        projected = point + t*drawing_plane_.normal;
        return true;
    }
    return false;
}

void SketchViewer::on_sketchmodeCheckBox_toggled(bool status)
{
    if (status == true)
        sketch_mode_ = kSketchDrawMode;
    else
        sketch_mode_ = kSketchIdleMode;
}

void SketchViewer::setTreeWidget(QTreeWidget *tree_widget)
{
    tree_widget_ = tree_widget;

    connect(
        tree_widget_, SIGNAL(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)), 
        this, SLOT(on_currentItem_changed(QTreeWidgetItem*, QTreeWidgetItem*)));

    connect(
        tree_widget_, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)), 
        this, SLOT(on_item_doubleclicked(QTreeWidgetItem*, int)));
}

void SketchViewer::setCurrentSketchIndex(int i)
{
    cur_sketch_index_ = i;

    if (i >= 0)
    {
        tree_widget_->setCurrentItem(tree_widget_->topLevelItem(i));
        // if (getCameraMode() == kCameraFirstPersonMode)
            switchFirstPersonCamera(currentSketch());

        if (terrain_features_per_sketch_.size() > i)
          silhouette_segments_ = terrain_features_per_sketch_[i];

    } else
    {
        tree_widget_->setCurrentItem(NULL);
        // if (getCameraMode() == kCameraFirstPersonMode)
            switchFirstPersonCamera(NULL);
    }

    updateSketches();
}

void SketchViewer::setCurrentStrokeIndex(int i)
{
    if (cur_sketch_index_ >= 0)
    {
        if (i >= 0)
        {
            cur_stroke_index_ = i;

            QTreeWidgetItem *top_item = tree_widget_->topLevelItem(cur_sketch_index_);
            tree_widget_->setCurrentItem(top_item->child(i));
        } else
            tree_widget_->setCurrentItem(NULL);

        updateSketches();
    } else
        tree_widget_->setCurrentItem(NULL);
}

void SketchViewer::addSketch(Sketch *sketch)
{
    if (sketch)
    {
        sketches_.push_back(sketch);
        int index = sketches_.size() - 1;
        QTreeWidgetItem *item = new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Sketch: %1").arg(index)));
        tree_widget_->addTopLevelItem(item);

        for (int m=0; m < sketch->numStrokes(); ++m)
        {
            addStroke(index, m);
        }
    }
}

void SketchViewer::addStroke(Sketch *sketch, Stroke *stroke)
{
    if (sketch)
    {
        addStroke(indexOf(sketch), sketch->indexOf(stroke));
    }
}

void SketchViewer::addStroke(int sketch_index, Stroke *stroke)
{
    addStroke(sketch_index, sketchAt(sketch_index)->indexOf(stroke));
}

void SketchViewer::addStroke(int sketch_index, int stroke_index)
{
    if (sketch_index >=0 && stroke_index >= 0)
    {
        QTreeWidgetItem *item = tree_widget_->topLevelItem(sketch_index);
        if (item)
        {
            item->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Stroke: %1").arg(stroke_index))));
        }
    }
}

void SketchViewer::on_currentItem_changed ( QTreeWidgetItem * current, QTreeWidgetItem * previous )
{
    if (current != previous)
    {
        if (current && !current->parent()) // a sketch is selected
        {
            std::cout << "Sketch changed " << tree_widget_->indexOfTopLevelItem(current) << "\n";
            setCurrentSketchIndex(tree_widget_->indexOfTopLevelItem(current));
            cur_stroke_index_ = -1;
        } else if (current && current->parent()) // a stroke is selected
        {
            QTreeWidgetItem *top_item = current->parent();
            cur_sketch_index_ = tree_widget_->indexOfTopLevelItem(top_item);
            int stroke_index = top_item->indexOfChild(current);

            std::cout << "Stroke changed " << cur_sketch_index_<< " " << stroke_index << "\n";
            setCurrentStrokeIndex(stroke_index);
        } else
        {
            std::cout << "Item changed " << -1 << "\n";
            cur_stroke_index_ = -1;
            cur_sketch_index_ = -1;
        }
    }
}

void SketchViewer::on_item_doubleclicked ( QTreeWidgetItem * item, int column )
{

}

void nurbsError(GLenum errorCode)
{
    const GLubyte *estring;
    estring = gluErrorString(errorCode);
    std::cerr << "Nurbs Error: " << estring << std::endl << std::flush;
}
