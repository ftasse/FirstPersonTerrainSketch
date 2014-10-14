#ifndef TERRAIN_H
#define TERRAIN_H

class QFileInfo;

#include <vector>
#include <eigen3/Eigen/Core>

class Terrain
{
public:

    //edges separating a front facing face from a back facing face
    struct SilhouetteSegment
    {
        std::vector<Eigen::Vector3f> points;
        bool isSilhouette;

        SilhouetteSegment(): isSilhouette(false)
        {}
    };

    typedef Eigen::Vector3f Normal;

    explicit Terrain(short width = 0, short height = 0);
    explicit Terrain(const Terrain& terrain, int rate);

    ~Terrain();

    inline short width() const { return width_; }
    inline short height() const { return height_; }
    inline float xscale() const { return 1; /*scale_[0];*/ }
    inline float max_altitude () const { return max_altitude_; }
    inline float min_altitude () const { return min_altitude_; }

    short getRelativeAltitude(int x, int y) const;
    void setRelativeAltitude(int x, int y, short relative_altitude);

    float getAltitude(int x, int y) const;     // use BaseHeight + Elevation * HeightScale / 65536
    void setAltitude(int x, int y, float altitude); // use BaseHeight + Elevation * HeightScale / 65536

    short getInterpolatedRelativeAltitude(float fx, float fy);
    float getInterpolatedAltitude(float fx, float fy);

    Normal getGridNormal(int i, int j) const;
    Normal getVertexNormal(int x, int y) const;
    bool hasGridNormals() const { return altitudes_ != NULL; }
    bool hasVertexNormals() const { return vertex_normals_ != NULL; }

    void setVertexNormalsFromRGBTexture(float *texture_data)
    {
        vertex_normals_ = texture_data;
    }

    bool validCoordinates(int x, int y) const;

    //void computeGridNormals();

    bool load(const char *path);
    bool save(const char *path) const;
    void updateMinMax();

    void copyFrom(const Terrain &rhs)
    {
      eraseData();
      for (int k = 0; k < 3; ++k)
        scale_[k] = rhs.scale_[k];
      height_scale_ = rhs.height_scale_;
      base_height_= rhs.base_height_;
      planet_radius_ = rhs.planet_radius_;
      render_mode_ = rhs.render_mode_;

      min_relative_altitude_ = rhs.min_relative_altitude_;
      min_altitude_ = rhs.min_altitude_;
      max_relative_altitude_ = rhs.max_relative_altitude_;
      max_altitude_ =rhs. max_altitude_;

      width_ = rhs.width_; height_ = rhs.height_;
      if (rhs.altitudes_)
      {
        altitudes_ = new float [width_*height_];
        memcpy(altitudes_, rhs.altitudes_, sizeof(float)*width_*height_);
      }
      if (rhs.vertex_normals_)
      {
        vertex_normals_ = new float [3*width_*height_];
        memcpy(vertex_normals_, rhs.vertex_normals_, sizeof(float)*3*width_*height_);
      }
    }

    std::string getPath() const { return path_; }

private:
    float *altitudes_;   // relative altitudes, in terrain units, row-major
    float *vertex_normals_;
    short width_;
    short height_;
    short height_scale_;
    short base_height_;
    float scale_[3];
    float planet_radius_; // in kilometers
    unsigned int render_mode_; // mode 0 for rendered flat and 1 for rendered draped/stretched

    short min_relative_altitude_, max_relative_altitude_;
    float min_altitude_, max_altitude_;
    std::string path_;

    inline int convertCoordinates(int x, int y) const;
    float relativeAltitudeToAbsolute(short relative_altitude) const;
    short absoluteAltitudeToRelative(float altitude) const;

    bool loadTerragen(const char *path);
    bool saveTerragen(const char *path) const;

    bool loadTiff(const char *path);
    bool saveTiff(const char *path) const;

    void setDefaultProperties();
    void eraseData();

    Terrain(const Terrain &rhs);
    bool operator =(const Terrain &rhs);
};

bool rayTerrainIntersect(Terrain *terrain,
                         Eigen::Vector3f start, Eigen::Vector3f ray_dir,
                         float max_dist, std::vector<Eigen::Vector3f> &intersects,
                         float precision = 0.01f);

#endif // TERRAIN_H
