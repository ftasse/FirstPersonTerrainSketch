#include "sketch.h"

#include <float.h>
#include <iostream>
#include <fstream>
#include <cctype>
#include <string>

#include <QFileInfo>
#include <QImage>
#include <QXmlStreamReader>

#include "tools/offline_terrain_renderer.h"
#include "algorithms/polyline_completion.h"

bool lessStroke(const Stroke *s1, const Stroke *s2)
{
    return s1->sort_id < s2->sort_id;
}

Sketch::Sketch():
    active_polyline_(NULL), cur_stroke_id_(-1)
{
}

Sketch::~Sketch()
{
  clear();
}

void Sketch::sortWithOrder(std::vector<int> sorted_stroke_ids)
{
    std::vector<Stroke *> tmp;
    for (unsigned int i = 0; i <sorted_stroke_ids.size(); ++i)
    {
        tmp.push_back(getStroke(sorted_stroke_ids[i]));
    }
    strokes_ = tmp;
}

void Sketch::clear()
{
  for (unsigned int i = 0; i < strokes_.size(); ++i)
      delete strokes_[i];
  strokes_.clear();
  cur_stroke_id_ = -1;
  delete active_polyline_;
  active_polyline_ = NULL;
}

int Sketch::numStrokes() const
{
    return strokes_.size();
}

Stroke* Sketch::getStroke(int idx)
{
    assert(idx >= 0 && idx < numStrokes());
    return strokes_[idx];
}

Stroke* Sketch::getCurrentStroke()
{
    return getStroke(cur_stroke_id_);
}

TerrSketch::Polyline* Sketch::getActivePolyline()
{
    return active_polyline_;
}

void Sketch::addStroke(Stroke *stroke)
{
    Stroke::Point start = stroke->getControlPoint(0);
    Stroke::Point end = stroke->getControlPoint(stroke->numControlPoints()-1);
    if (start[0] > end[0])
        stroke->invert();

    stroke->cleanup();
    stroke->updateKnots();
    strokes_.push_back(stroke);
    cur_stroke_id_ = strokes_.size()-1;
}

void Sketch::startStroke()
{
    delete active_polyline_;
    active_polyline_ = new TerrSketch::Polyline ();
}

void Sketch::completeEnds()
{
    std::vector< std::vector<Eigen::Vector3f> > poly_points (numStrokes());

    for (unsigned int i = 0; i < strokes_.size(); ++i)
        strokes_[i]->getPoints(poly_points[i]);

    PolylineCompletion completion;
    completion.setPolylines(poly_points);
    completion.completePolylines();
    completion.getCompletedPolylines(poly_points);

    for (unsigned int i = 0; i < strokes_.size(); ++i)
        strokes_[i]->setPoints(poly_points[i]);
}

bool Sketch::stopStroke()
{
    bool added = false;

    if (active_polyline_ != NULL)
    {
        if (active_polyline_->numEdges() > 1)
        {
            Stroke *new_stroke = new Stroke();
            new_stroke->convertFromPolyline(*active_polyline_);
            addStroke(new_stroke);
            added = true;
        }
        delete active_polyline_;
        active_polyline_ = NULL;
    }

    return added;
}

void Sketch::addPolylinePoint(Stroke::Point point)
{
    active_polyline_->addPoint(point);
}

bool Sketch::load(const char *path)
{
    QFileInfo finfo(path);

    if (!finfo.exists())
    {
        std::cerr << "Error: sketch file does not exists at " << finfo.absoluteFilePath().toStdString()
                  << "\n" << std::flush;
        return false;
    } else if (QString::compare(finfo.suffix(), "vect", Qt::CaseInsensitive) == 0)
    {
        return loadVect(path);
    } else
    {
        std::cerr << "Error: sketch file format " << finfo.suffix().toStdString()
                  << "is not supported.\n" << std::flush;
        return false;
    }
}

bool Sketch::save(const char *path)
{
    QFileInfo finfo(path);

    if (QString::compare(finfo.suffix(), "vect", Qt::CaseInsensitive) == 0)
    {
        return saveVect(path);
    } else
    {
        std::cerr << "Error: sketch file format " << finfo.suffix().toStdString()
                  << " is not supported.\n" << std::flush;
        return false;
    }
}

// see http://www.geomview.org/docs/oogltour.html for VECT specification

bool Sketch::loadVect(const char *path)
{
    for (unsigned int i = 0; i < strokes_.size(); ++i)
        delete strokes_[i];
    strokes_.clear();

    delete active_polyline_;
    active_polyline_ = NULL;

    std::ifstream ifs(path);

    if (!ifs)
    {
        std::cerr << "Error: could not open sketch file for reading at " << path
                  << "\n" << std::flush;
        return false;
    } else
    {
        bool succeded = false;
        std::string buffer;

        ifs >> buffer;
        if (buffer.compare("VECT") != 0)
        {
            std::cerr << "Error: This VECT is invalid since it does not starts with VECT"
                      << "\n" << std::flush;
            succeded = false;
        } else
        {
            int num_strokes, total_num_points, num_colors;
            ifs >> num_strokes >> total_num_points >> num_colors;

            std::vector<int> stroke_point_counts(num_strokes, 0);
            for (int i = 0; i < num_strokes; ++i)
                ifs >> stroke_point_counts[i];

            int color_count;
            for (int i = 0; i < num_strokes; ++i)
                ifs >> color_count;

            for (int i = 0; i < num_strokes; ++i)
            {
                Stroke *new_stroke = new Stroke ();
                new_stroke->read_text(ifs, stroke_point_counts[i]);
                addStroke(new_stroke);
            }

            //get colors
            int color[4];
            for (int i = 0; i < num_colors; ++i)
                ifs >> color[0] >> color[1] >> color[2] >> color[3];

            //get view frame
            camera_info_.read_text(ifs);
//            ifs >> view_position_[0] >> view_position_[1] >> view_position_[2];
//            ifs >> view_direction_[0] >> view_direction_[1] >> view_direction_[2];

            succeded = true;
        }

        ifs.close();
        return succeded;
    }

}

bool Sketch::saveVect(const char *path)
{
    if (numStrokes() <= 0)
    {
        std::cerr << "Error: sketch has no strokes. Cannot save to " << path
                  << "\n" << std::flush;
        return false;
    }

    std::ofstream ofs(path);

    if (!ofs)
    {
        std::cerr << "Error: could not open sketch file for writing at " << path
                  << "\n" << std::flush;
        return false;
    } else
    {
        int total_num_points = 0;

        for (unsigned int i = 0; i < strokes_.size(); ++i)
            if (strokes_[i]->closed())
                total_num_points += strokes_[i]->numControlPoints()+1;
            else
                total_num_points += strokes_[i]->numControlPoints();

        ofs << "VECT" << std::endl;
        ofs << numStrokes() << " " << total_num_points << " " << "1" << std::endl;

        //  list num of points per stroke
        for (unsigned int i = 0; i < strokes_.size(); ++i)
            if (strokes_[i]->closed())
                ofs << strokes_[i]->numControlPoints()+1 << " ";
            else
                ofs << strokes_[i]->numControlPoints() << " ";
        ofs << std::endl;

        // list num colors per stroke
        ofs << "1";
        for (unsigned int i = 1; i < strokes_.size(); ++i)
            ofs << " 0";
        ofs << std::endl;

        // list points
        ofs << std::endl;
        for (unsigned int i = 0; i < strokes_.size(); ++i)
        {
            strokes_[i]->write_text(ofs);
        }

        //list colors
        ofs << std::endl;
        ofs << "0 0 0 1" << std::endl;

        //list view frame
        camera_info_.write_text(ofs);
//        ofs << view_position_[0] << " " << view_position_[1] << " " << view_position_[2] << "\n";
//        ofs << view_direction_[0] << " " << view_direction_[1] << " " << view_direction_[2] << "\n";
    }

    ofs.close();
    return true;
}

bool Sketch::loadSvg(const char *path, const char *terrain_path)
{
  Terrain *terrain = NULL;
  if (terrain_path != NULL)
    {
      terrain = new Terrain ();
      if (!terrain->load(terrain_path))
        {
          delete terrain; terrain = NULL;
        }
    }
  bool success = loadSvg(path, terrain);
  delete terrain;
  return success;
}

bool Sketch::loadSvg(const char *path, Terrain *terrain)
{
    QFile file(path);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
      qWarning("Cannot open file");
      return false;
    }
    QString localName;
    QXmlStreamAttributes attributes;
    QXmlStreamReader *pXml = new QXmlStreamReader(&file);
    pXml->setNamespaceProcessing(false);
    while (!pXml->atEnd()) {
      switch (pXml->readNext()) {
          case QXmlStreamReader::StartElement:
            localName = pXml->name().toString();
            if (localName.compare("svg") == 0) {
                attributes = pXml->attributes();
                std::stringstream ss(attributes.value("viewBox").toString().toStdString());
                Eigen::Vector2i origin;
                ss >> origin[0] >> origin[1] >> camera_info_.screen_width >> camera_info_.screen_height;
            }
            if (localName.compare("path") == 0) {
              attributes = pXml->attributes();
              QStringRef data  = attributes.value("d");

              std::vector<Eigen::Vector2i> control_points;
              std::vector<Eigen::Vector2i> cur_points;
              int count = data.count();
              std::string cur_str;
              Eigen::Vector2i cur_point;

              for (int i = 0; i < count; ++i)
              {
                  char c = data.at(i).toLatin1();
                  switch (c) {
                    case 'M':
                    case 'C':
                      control_points.insert(control_points.end(),
                                            cur_points.begin(),
                                            cur_points.end());
                      cur_str.clear();
                      cur_points.clear();
                      break;
                    case ',':
                      cur_point[0] = round(atof(cur_str.c_str()));
                      cur_str.clear();
                      break;
                    case ' ':
                    case '\n':
                    case '\t':
                      if (cur_str.size() > 0)
                        {
                          cur_point[1] = camera_info_.screen_height-1-round(atof(cur_str.c_str()));
                          cur_points.push_back(cur_point);
                          cur_str.clear();
                        } else
                        {
                          control_points.insert(control_points.end(),
                                                cur_points.begin(),
                                                cur_points.end());
                          cur_str.clear();
                          cur_points.clear();
                        }
                      break;
                    default:
                      cur_str += c;
                      break;
                    }
              }
              if (cur_str.size() > 0)
                {
                  cur_point[1] = camera_info_.screen_height-1-atof(cur_str.c_str());
                  cur_points.push_back(cur_point);
                  control_points.insert(control_points.end(),
                                        cur_points.begin(),
                                        cur_points.end());
                  cur_str.clear();
                  cur_points.clear();
                }

              if (control_points.size() <= 2)
                continue;

              Stroke *stroke = new Stroke ();
              stroke->set_identifier(attributes.value("id").toString().toStdString());
              if (terrain == NULL)
                {
                  for (unsigned int i = 0; i < control_points.size(); ++i)
                    stroke->addControlPoint(Stroke::Point(control_points[i][0],  0.0f, control_points[i][1]));
                } else
                {
                  OfflineTerrainRenderer terrain_renderer(terrain);
                  terrain_renderer.setCameraInfo(camera_info());
                  std::vector<Eigen::Vector3f> unprojected_points;
                  terrain_renderer.unProject(control_points, unprojected_points);
                  for (unsigned int i = 0; i < unprojected_points.size(); ++i)
                    stroke->addControlPoint(unprojected_points[i]);
                }
              addStroke(stroke);
            }
            break;
         default:
            break;
      }
    }

    return true;
}

bool Sketch::saveAsHeightMap(const char *path, Terrain *terrain)
{
  int width = 512, height = 512; //TODO Remove hard coded size
  float minh = FLT_MAX, maxh = -FLT_MAX;

  for (int i = 0; i < numStrokes(); ++i)
    {
      for (int j = 0; j < strokes_[i]->numControlPoints(); ++j)
        {
          Stroke::Point point = strokes_[i]->getControlPoint(j);
          int x = ceil(point[0]) + 5;
          int y = ceil(point[1]) + 5;
          width = std::max(width, x);
          height = std::max(height, y);

          minh = std::min(point[2], minh);
          maxh = std::max(point[2], maxh);
        }
    }

  if (terrain != NULL)
    {
      minh = std::min(terrain->min_altitude(), minh);
      maxh = std::max(terrain->max_altitude(), maxh);
      width = terrain->width();
      height = terrain->height();
    }

  QImage image(width, height, QImage::Format_Indexed8);
  uchar *data = image.bits();

  if (terrain == NULL)
    {
      maxh += 5.0f;
      for (int i = 0; i < width*height; ++i)
        data[i] = 255;
    }
  else
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x)
      {
        int i = y*width + x;
        float percent = (terrain->getAltitude(x, y)-minh)/(maxh-minh);
        data[i] = (unsigned char)(percent*255);
      }

  for (int i = 0; i < numStrokes(); ++i)
    {
      for (int j = 0; j < strokes_[i]->numControlPoints(); ++j)
        {
          Stroke::Point point = strokes_[i]->getControlPoint(j);
          int x = ceil(point[0]); int y = ceil(point[1]);
          float percent = (point[2]-minh)/(maxh-minh);

          if (terrain != NULL) data[y*width + x] = 0.0f;
          else data[y*width + x] = (unsigned char)(percent*255);
        }
    }

  QVector<QRgb> grayscale;
  for (int i = 0; i < 256; ++i)  grayscale.append(qRgb(i, i, i));
  image.setColorTable(grayscale);
  image.save(path);

  return true;
}

void Sketch::setDefaultCamera(Eigen::Vector2i terrain_size)
{
    CameraInfo info = camera_info();
    info.position = Eigen::Vector3f(terrain_size[0]/2.0, -terrain_size[1]*0.20-200.0, 5.0);
    info.direction = Eigen::Vector3f(0.0, 1.0, 0.0).normalized();
    info.up_vector = Eigen::Vector3f(0.0, 0.0, 1.0);
    setCameraInfo(info);

    float scale_factor = std::min(terrain_size[0]/((float) info.screen_width),
                                  terrain_size[1]/((float) info.screen_height));

    for (int i = 0; i < numStrokes(); ++i )
    {
        Stroke *stroke = getStroke(i);
        for (int j = 0; j < stroke->numControlPoints(); ++j)
        {
            Stroke::Point point = stroke->getControlPoint(j);
            point *= scale_factor;
            point[1] = terrain_size[1]*0.80;
            stroke->setControlPoint(j, point);
        }
    }
}
