#ifndef SKETCH_H
#define SKETCH_H

#include "geometry/stroke.h"
#include "tools/camera_info.h"

class QFileInfo;
class Terrain;

class Sketch
{
public:
    Sketch();
    ~Sketch();

    int numStrokes() const;
    Stroke* getStroke(int idx);
    Stroke* getCurrentStroke();
    TerrSketch::Polyline* getActivePolyline();

    int getCurrentStrokeId()
    {
        return cur_stroke_id_;
    }

    void deleteStroke(int stroke_index)
    {
        delete strokes_[stroke_index];
        strokes_.erase(strokes_.begin() + stroke_index);
    }

    CameraInfo camera_info() const { return camera_info_; }
    void setCameraInfo(CameraInfo camera_info) { camera_info_ = camera_info; }

    void startStroke();
    bool stopStroke();
    void addPolylinePoint(Stroke::Point point);

    void sortWithOrder(std::vector<int> sorted_stroke_ids);

    void clear();

    bool load(const char *path);
    bool save(const char *path);

    bool loadSvg(const char *path, const char *terrain_path);
    bool loadSvg(const char *path, Terrain *terrain = NULL);
    bool saveAsHeightMap(const char *path, Terrain *terrain = NULL);

    void completeEnds();

    void setDefaultCamera(Eigen::Vector2i terrain_size = Eigen::Vector2i(512, 512));

    int indexOf(Stroke* stroke)
    {
        std::vector<Stroke *>::iterator it = std::find(strokes_.begin(), strokes_.end(), stroke);
        if (it != strokes_.end())
        {
            return std::distance( strokes_.begin(), it);
        } else
            return -1;
    }

private:
    std::vector<Stroke *> strokes_;
    TerrSketch::Polyline *active_polyline_;
    int cur_stroke_id_;

    CameraInfo camera_info_;

    bool loadVect(const char *path);
    bool saveVect(const char *path);

    void addStroke(Stroke *stroke);
};

#endif // SKETCH_H
