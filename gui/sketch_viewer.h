#ifndef SKETCH_VIEWER_H
#define SKETCH_VIEWER_H

#include <QTreeWidget>

#include "gui/terrain_viewer.h"
#include "geometry/sketch.h"
#include "tools/deformation_weighting.h"

struct DeformationWeighting;

class SketchViewer : public TerrainViewer
{
    Q_OBJECT

    struct DrawingPlane
    {
        Eigen::Vector3f normal;
        Eigen::Vector3f point_on_plane;
    };

    typedef TerrainViewer Base;
public:
    typedef enum {
        kSketchIdleMode = 0,
        kSketchDrawMode = 1
    } SketchMode;

    explicit SketchViewer(QWidget *parent = 0);
    ~SketchViewer();

    SketchMode getSketchMode() const { return sketch_mode_; }
    void setSketchMode(SketchMode sketch_mode) { sketch_mode_ = sketch_mode; }

    bool loadSketch(const char *sketch_path);
    bool saveSketch(const char *sketch_path, Sketch *sketch = NULL);
    void updateSketches();

    void makeSketchPlanar(Sketch* sketch);

    void updateTerrain(bool update_person);

    void deleteSketch(Sketch *sketch = NULL)
    {
        if (sketch == NULL)
            deleteSketch(cur_sketch_index_);
        else
            deleteSketch(indexOf(sketch));
    }

    void deleteSketch(int sketch_index)
    {
        Sketch *sketch = sketchAt(sketch_index);
        if (sketch)
        {
            delete sketches_[sketch_index];
            sketches_.erase(sketches_.begin() + sketch_index);
            tree_widget_->takeTopLevelItem(sketch_index);

            if (terrain_features_per_sketch_.size() > sketch_index)
              terrain_features_per_sketch_.erase(terrain_features_per_sketch_.begin()+sketch_index);

            cur_stroke_index_ = -1;
            setCurrentSketchIndex(-1);
            updateSketches();
        }
    }

    void deleteStroke(int sketch_index, int stroke_index);

    void deleteSelected()
    {
        std::cout << "cur_stroke_index_: " << cur_stroke_index_ << "\n";

        if (cur_stroke_index_ >= 0)
        {
            std::cout << "Delete stroke: " << cur_stroke_index_ << "\n";
            deleteStroke(cur_sketch_index_, cur_stroke_index_);
            reloadSketches();
        } else if (cur_sketch_index_ >= 0)
        {
            std::cout << "Delete sketch: " << cur_sketch_index_ << "\n";
            deleteSketch(cur_sketch_index_);
            reloadSketches();
        }
    }

    void reloadSketches()
    {
        std::vector<Sketch*> tmp = sketches_;
        cur_sketch_index_ = -1;
        cur_stroke_index_ = -1;
        sketches_.clear();
        tree_widget_->clear();

        for (unsigned int i = 0; i < tmp.size(); ++i)
            addSketch(tmp[i]);
    }

    void startNewSketch()
    {
        if (getCameraMode() == kCameraFirstPersonMode)
            first_person_camerainfo_ = getCurrentCameraInfo();

        sketch_mode_ = kSketchDrawMode;
        addSketch(new Sketch ());
        sketches_.back()->setCameraInfo(first_person_camerainfo_);
        setCurrentSketchIndex(sketches_.size() - 1);
    }

    void stopNewSketch()
    {
        sketch_mode_ = kSketchIdleMode;
    }

    void deformTerrain();
    void lowerSilhouettes();
    void embedSketches();

    void matchStrokesToFeatures();

    void extractSilhouetteEdges();
    void extractRidges();
    void extractFeatures();

    void setWeighting(DeformationWeighting weighting)
    {
        weighting_ = weighting;
    }

    DeformationWeighting weighting()
    {
        return weighting_;
    }

    void getSketchCameraInfos(std::vector<CameraInfo> &camera_infos)
    {
      camera_infos.clear();
      for (unsigned int i = 0; i < sketches_.size(); ++i)
        camera_infos.push_back(sketches_[i]->camera_info());
    }

    void adjustFirstPersonCamera()
    {
      if (cur_first_person_camera_locked_)
      {
          CameraInfo tmp = first_person_camerainfo_;
          first_person_camerainfo_ = saved_first_person_camerainfo_;
          TerrainViewer::adjustFirstPersonCamera();
          saved_first_person_camerainfo_ = first_person_camerainfo_;
          first_person_camerainfo_ = tmp;
          updateCamera();
      } else
        TerrainViewer::adjustFirstPersonCamera();
    }

    void setTreeWidget(QTreeWidget *tree_widget = NULL);
    void setCurrentSketchIndex(int i);
    void setCurrentStrokeIndex(int i);
    void addSketch(Sketch *sketch);
    void addStroke(Sketch *sketch, Stroke *stroke);
    void addStroke(int sketch_index, Stroke *stroke);
    void addStroke(int sketch_index, int stroke_index);

protected:
    void init();
    void draw();

    void keyPressEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

	QString helpString() const;

    Sketch* sketchAt(unsigned int id)
    {
        if (id < 0 || id >= sketches_.size())  return NULL;
        else return sketches_[cur_sketch_index_];
    }

    Sketch* currentSketch()
    {
        return sketchAt(cur_sketch_index_);
    }

    Stroke* currentStroke()
    {
        Sketch* sketch =  currentSketch();
        if (!sketch) return NULL;
        else return sketch->getCurrentStroke();
    }

    int indexOf(Sketch* sketch)
    {
        std::vector<Sketch*>::iterator it = std::find(sketches_.begin(), sketches_.end(), sketch);
        if (it != sketches_.end())
        {
            return std::distance( sketches_.begin(), it);
        } else
            return -1;
    }

    bool hasSketches()
    {
        return sketches_.size() > 0;
    }

    void switchFirstPersonCamera(Sketch* sketch = NULL);

    int cur_sketch_index_;
    int cur_stroke_index_;
    QTreeWidget *tree_widget_;

private:
    std::vector<Sketch *> sketches_;
    bool is_sketching_;
    SketchMode sketch_mode_;
    DrawingPlane drawing_plane_;

    CameraInfo saved_first_person_camerainfo_;

    double modeview_matrix_[16];
    double projection_matrix_[16];
    int viewport_[4];

    DeformationWeighting weighting_;

    void startSketching(QPoint start_pos);
    void stopSketching();
    void addToSketch(const Stroke::Point worldCoords);

    bool pixelToWorldCoords(const QPoint mousePos,
                            Stroke::Point &world_coords);
    bool projectOnDrawingPlane(const Stroke::Point point,
                               Stroke::Point &projected,
                               bool orthographic_camera = false);
    void drawSketch(Sketch* sketch);
    void drawStroke(Sketch* sketch, Stroke *stroke);
    void drawPolyline(TerrSketch::Polyline *polyline);

private:
    GLUnurbs *nurbs_;
    std::vector<Eigen::Vector3f> picked_sihouette_points_;
    std::vector<Eigen::Vector3f> picked_sihouette_colors_;

    std::vector<Terrain::SilhouetteSegment> constraint_edges_;
    std::vector<Terrain::SilhouetteSegment> constraint_edges_on_terrain_;

    std::vector< std::vector<Terrain::SilhouetteSegment> > terrain_features_per_sketch_;

public slots:
    void on_sketchmodeCheckBox_toggled(bool status);

    void on_currentItem_changed ( QTreeWidgetItem * current, QTreeWidgetItem * previous );
    void on_item_doubleclicked ( QTreeWidgetItem * item, int column );
};

class LowerSilhouettesWorker : public QObject
{
    Q_OBJECT
public:
    LowerSilhouettesWorker(Terrain *terrain, std::vector<Sketch*> sketches, std::vector<Terrain::SilhouetteSegment> &constraint_edges):
        terrain_(terrain), sketches_(sketches), constraint_edges_(constraint_edges) {}

    ~LowerSilhouettesWorker()
    {
        terrain_ = NULL;
    }

signals:
    void sendUpdateTerrain(bool);

public slots:
    void doWork();

private:
    Terrain *terrain_;
    std::vector<Sketch *> sketches_;
    std::vector<Terrain::SilhouetteSegment> &constraint_edges_;
};

#endif // SKETCH_VIEWER_H
