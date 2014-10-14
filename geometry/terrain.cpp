#include "terrain.h"

#if __unix__
#include <endian.h>
#elif _WIN32
//#include <endian.h>
#define htole32 // (host to little-endian encoding)
#define le32toh // (little-endian to host encoding)
#define htole16 // (host to little-endian encoding)
#define le16toh // (little-endian to host encoding)
#define htole64 // (host to little-endian encoding)
#define le64toh // (little-endian to host encoding)
#endif

#include <stdio.h>
#include <float.h>
#include <math.h>

#ifndef _WIN32
#include <inttypes.h>
#else
#include <cstdint>
#endif

#include <cassert>
#include <iostream>
#include <fstream>

#ifdef TIFF_LIB_EXISTS
#include <tiffio.h>
#endif

#include <QFileInfo>
#include <eigen3/Eigen/Geometry>

#define kTIFFTAG_TERRAINSCALE  65400
#define kTIFFTAG_TERRAINHEIGHTSCALE 65401
#define kTIFFTAG_TERRAINBASEHEIGHT 65402
#define kTIFFTAG_TERRAINPLANETRADIUS 65403
#define kTIFFTAG_TERRAINRENDERMODE 65404
#define kTIFFTAG_TERRAINMINALTITUDE 65405
#define kTIFFTAG_TERRAINALTRANGE 65406

#ifdef TIFF_LIB_EXISTS
static void _XTIFFInitialize();
#endif

inline short read_int16(char *buffer)
{
    return le16toh(*((short *) buffer));
}

inline void write_int16(std::ofstream &ofs, short x)
{
    short x_ = htole16(x);
    ofs.write((char*) &x_, 2);
}

inline int32_t read_int32(char *buffer)
{
    return le32toh(*((int32_t *) buffer));
}

inline void write_int32(std::ofstream &ofs, short x)
{
    short x_ = htole32(x);
    ofs.write((char*) &x_, 4);
}

inline int64_t read_int64(char *buffer)
{
    return le64toh(*((int64_t *) buffer));
}

inline void write_int64(std::ofstream &ofs, short x)
{
    short x_ = htole64(x);
    ofs.write((char*) &x_, 8);
}

Terrain::Terrain(short width, short height) :
    width_(width), height_(height)

{
#ifdef TIFF_LIB_EXISTS
    _XTIFFInitialize();
#endif

    setDefaultProperties();
    altitudes_ = NULL;
    vertex_normals_ = NULL;
    if (width_*height_ > 0)
    {
        altitudes_ = new float [width_*height_];
        for (int i = 0; i < width_*height_; ++i)
            altitudes_[i] = 0.0f;
        updateMinMax();
    }
}

Terrain::Terrain(const Terrain& terrain, int rate)
{
    width_= terrain.width()/rate;
    height_ = terrain.height()/rate;
    min_altitude_ = terrain.min_altitude()/rate;
    max_altitude_ = terrain.max_altitude()/rate;
    vertex_normals_ = NULL;
    altitudes_ = NULL;

    altitudes_ = new float [width_*height_];
    for(int i = 0; i < width(); i++)
    {
        for(int j = 0; j < height(); j++)
            setAltitude(i,j, terrain.getAltitude(i*rate,j*rate));
    }

    if (terrain.hasVertexNormals())
    {
        vertex_normals_ = new float [3*width_*height_];
        for(int i = 0; i < width(); i++)
        {
            for(int j = 0; j < height(); j++)
            {
                Eigen::Vector3f normal = terrain.getVertexNormal(i*rate,j*rate);
                vertex_normals_[3*convertCoordinates(i,j) + 0] = normal[0];
                vertex_normals_[3*convertCoordinates(i,j) + 1] = normal[1];
                vertex_normals_[3*convertCoordinates(i,j) + 2] = normal[2];
            }
        }
    }
}

Terrain::~Terrain()
{
    eraseData();
}

short Terrain::getRelativeAltitude(int x, int y) const
{
    return absoluteAltitudeToRelative(getAltitude(x, y));
}

void Terrain::setRelativeAltitude(int x, int y, short relative_altitude)
{
    setAltitude(x, y, relativeAltitudeToAbsolute(relative_altitude));
}

float Terrain::getAltitude(int x, int y) const
{
    assert(validCoordinates(x, y));
    return altitudes_[convertCoordinates(x, y)];
}

void Terrain::setAltitude(int x, int y, float altitude)
{
    assert(validCoordinates(x, y));
    altitudes_[convertCoordinates(x, y)] = altitude;
}

short Terrain::getInterpolatedRelativeAltitude(float fx, float fy)
{
    return absoluteAltitudeToRelative(getInterpolatedAltitude(fx, fy));

}

float Terrain::getInterpolatedAltitude(float fx, float fy)
{
    int xmin = floor(fx), ymin = floor(fy);
    if (xmin < 0 && ymin >= 0)
        return getAltitude(0, ymin);
    else if (xmin >= 0 && ymin < 0)
        return getAltitude(xmin, 0);
    else if (xmin < 0 && ymin < 0)
        return getAltitude(0, 0);
    else if (xmin >= width()-1 && ymin < height()-1)
        return getAltitude(width()-1, ymin);
    else if (xmin < width()-1 && ymin >= height()-1)
        return getAltitude(xmin, height()-1);
    else if (xmin >= width()-1 && ymin >= height()-1)
        return getAltitude(width()-1, height()-1);

    float dx = fx-xmin, dy = fy-ymin;
    return (1-dx)*((1-dy)*getAltitude(xmin, ymin) + dy*getAltitude(xmin, ymin+1))
            + dx*((1-dy)*getAltitude(xmin+1, ymin) + dy*getAltitude(xmin+1, ymin+1));

}

float Terrain::relativeAltitudeToAbsolute(short relative_altitude) const
{
    return base_height_ + height_scale_*relative_altitude/65356.0f;
}

short Terrain::absoluteAltitudeToRelative(float altitude) const
{
    short val = static_cast<short> ((altitude - base_height_)*65356.0f/height_scale_);
    return val;
}

Terrain::Normal Terrain::getGridNormal(int i, int j) const
{
    assert(i >= 0 && j >= 0 && i < width_-1 && j < height_-1);

    Eigen::Vector3f vec1, vec2, normal;
    vec1 = Eigen::Vector3f(i+1, j+1, getAltitude(i+1, j+1)) - Eigen::Vector3f(i, j, getAltitude(i, j));
    vec2 = Eigen::Vector3f(i, j+1, getAltitude(i, j+1)) - Eigen::Vector3f(i+1, j, getAltitude(i+1, j));
    normal = vec1.cross(vec2);
    return normal.normalized();

//    Normal normal = getVertexNormal(i, j);
//    normal += getVertexNormal(i+1, j);
//    normal += getVertexNormal(i+1, j+1);
//    normal += getVertexNormal(i, j+1);
//    return normal.normalized();
}

Terrain::Normal Terrain::getVertexNormal(int x, int y) const
{
    assert(validCoordinates(x, y));
    int pos = 3*convertCoordinates(x, y);
    return Normal(vertex_normals_[pos], vertex_normals_[pos+1], vertex_normals_[pos+2]);
}

void Terrain::updateMinMax()
{
    min_relative_altitude_ = SHRT_MAX;
    max_relative_altitude_ = SHRT_MIN;
    min_altitude_ = FLT_MAX;
    max_altitude_ = -FLT_MAX;

    for (int i = 0; i < width_*height_; ++i)
    {
        min_altitude_ = std::min(min_altitude_, altitudes_[i]);
        max_altitude_ = std::max(max_altitude_, altitudes_[i]);
    }

    //TODO: Compute this normally
    float hrange = max_altitude_ - min_altitude_;
    float hbase = min_altitude_ + (hrange / 2.0f); // midpoint of hrange in terrain coordinates

    base_height_ = hbase;
    height_scale_ = ceil(hrange+1);

    min_relative_altitude_ = absoluteAltitudeToRelative(min_altitude_);
    max_relative_altitude_ = absoluteAltitudeToRelative(max_altitude_);
}

int Terrain::convertCoordinates(int x, int y) const
{
    return y*width_ + x;
}

bool Terrain::validCoordinates(int x, int y) const
{
    return x >= 0 && y >= 0 && x < width_ && y < height_;
}

void Terrain::setDefaultProperties()
{
    scale_[0] = scale_[1] = scale_[2] = 30.0f;
    height_scale_ = 65356/2;
    base_height_= 0;
    planet_radius_ = 6370.0f;
    render_mode_ = 0;

    min_relative_altitude_ = 0;
    min_altitude_ = 0;
}

void Terrain::eraseData()
{
    width_ = height_ = 0;
    delete [] altitudes_;
    altitudes_ = NULL;
    delete [] vertex_normals_;
    vertex_normals_ = NULL;
}

//void Terrain::computeGridNormals()
//{
//    if ((width_-1)*(height_-1) > 0)
//    {
//        delete [] grid_normals_;
//        grid_normals_ = new Normal [(width_-1)*(height_-1)];

//        for (int j = 0; j < height()-1; ++j)
//            for (int i = 0; i < width()-1; ++i)
//            {
//                Eigen::Vector3f vec1, vec2, normal;
//                vec1 = Eigen::Vector3f(i+1, j+1, getAltitude(i+1, j+1)) - Eigen::Vector3f(i, j, getAltitude(i, j));
//                vec2 = Eigen::Vector3f(i, j+1, getAltitude(i, j+1)) - Eigen::Vector3f(i+1, j, getAltitude(i+1, j));
//                normal = vec1.cross(vec2);
//                grid_normals_[j*(width()-1) + i] = normal.normalized();
//            }
//    }
//}

bool Terrain::load(const char *path)
{
    QFileInfo finfo(path);
    bool succeded = false;

    if (!finfo.exists())
    {
        std::cerr << "File does not exists at " << finfo.absoluteFilePath().toStdString()
                  << "\n" << std::flush;
    } else if (finfo.size() == 0)
    {
        std::cerr << "File has size = 0 bytes. Path: " << finfo.absoluteFilePath().toStdString()
                  << "\n" << std::flush;
    } else if (QString::compare(finfo.suffix(), "ter", Qt::CaseInsensitive) == 0)
    {
        succeded = loadTerragen(path);
    } else if (QString::compare(finfo.suffix(), "tif", Qt::CaseInsensitive) == 0)
    {
        succeded = loadTiff(path);
    } else
        std::cerr << "Terrain file format ." << finfo.suffix().toStdString()
                  << " is not supported.\n" << std::flush;

    if (succeded)
    {
        updateMinMax();
        path_ = finfo.absoluteFilePath().toStdString();
    }
    return succeded;
}

bool Terrain::save(const char *path) const
{
    QFileInfo finfo(path);
    bool succeded = false;

    if (QString::compare(finfo.suffix(), "ter", Qt::CaseInsensitive) == 0)
    {
        succeded = saveTerragen(path);
    } else if (QString::compare(finfo.suffix(), "tif", Qt::CaseInsensitive) == 0)
    {
        succeded = saveTiff(path);
    } else
        std::cerr << "Terrain file format ." << finfo.suffix().toStdString()
                  << " is not supported.\n" << std::flush;

    return succeded;
}

bool Terrain::loadTerragen(const char *path)
{
    setDefaultProperties();
    eraseData();

    char buffer[16] = "";
    char padding[8] = "";
    bool succeded = false;

    std::ifstream ifs;
    ifs.open(path, std::ios_base::in | std::ios_base::binary);
    if (!ifs.is_open())
    {
        std::cerr << "Error opening file at: " << path << "\n" << std::flush;
    } else
    {
        ifs.read(buffer, 16);
        if (strncmp(buffer, "TERRAGENTERRAIN ", 16) != 0)
            std::cerr << "Terragen file does not start with TERRAGENTERRAIN: "
                      << path << "\n" << std::flush;
        else
        {
            while (ifs.read(buffer, 4) && strncmp(buffer, "ALTW", 4) != 0)
            {
                //std::cout << std::string(buffer, 4) << "\n" << std::flush;

                if (strncmp(buffer, "SIZE", 4) == 0)
                {
                    ifs.read(buffer, 2);
                    short size = read_int16(buffer) + 1;
                    width_ = height_ = size;
                    ifs.read(padding, 2);
                } else if (strncmp(buffer, "XPTS", 4) == 0)
                {
                    ifs.read(buffer, 2);
                    width_ = read_int16(buffer);
                    ifs.read(padding, 2);
                } else if (strncmp(buffer, "YPTS", 4) == 0)
                {
                    ifs.read(buffer, 2);
                    height_ = read_int16(buffer);
                    ifs.read(padding, 2);
                } else if (strncmp(buffer, "SCAL", 4) == 0)
                {
                    ifs.read(buffer, 4);
                    scale_[0] = *((float *) buffer);
                    ifs.read(buffer, 4);
                    scale_[1] = *((float *) buffer);
                    ifs.read(buffer, 4);
                    scale_[2] = *((float *) buffer);
                } else if (strncmp(buffer, "CRAD", 4) == 0)
                {
                    ifs.read(buffer, 4);
                    planet_radius_ = *((float *) buffer);
                } else if (strncmp(buffer, "CRVM", 4) == 0)
                {
                    ifs.read(buffer, 4);
                    render_mode_ = *((unsigned int *) buffer);
                }
            }

            if (!ifs)
            {
                std::cerr << "Terragen file does not have altitude information: "
                          << path << "\n" << std::flush;
            } else if (width_ == 0 || height_ == 0)
            {
                std::cerr << "Terragen file has invalid size: "
                          << width_ << "x" << height_ << "\n" << std::flush;
            }
            else
            {
                ifs.read(buffer, 2);
                height_scale_ = read_int16(buffer);
                ifs.read(buffer, 2);
                base_height_ = read_int16(buffer);

                altitudes_ = new float [width_*height_];

                char tmp_buffer[2];
                for (int i = 0; i < width_*height_; ++i)
                {
                    ifs.read(tmp_buffer, 2);
                    altitudes_[i] = relativeAltitudeToAbsolute(read_int16(tmp_buffer));
                }

                if (!ifs) std::cerr << "Terragen file does not have all altitude values. \n" << std::flush;
                else succeded = true;
            }
        }
        ifs.close();
    }

    if (!succeded)
    {
        eraseData();
    }

    //if (succeded) for (int i = 0; i < 10; ++i) std::cout << altitudes_[i] << " " << std::flush;
    return succeded;
}

bool Terrain::saveTerragen(const char *path) const
{
    short size = (width_ == height_)?width_-1:width_*height_-1;

    std::ofstream ofs;
    ofs.open(path, std::ios_base::out | std::ios_base::binary);

    if (!ofs)
    {
        std::cerr << "Error opening Terragen image for writing: " << path << "\n" << std::flush;
        return false;
    }

    ofs.write("TERRAGENTERRAIN ", 16);

    ofs.write("SIZE", 4);
    write_int16(ofs, size); ofs.write("", 2);
    ofs.write("XPTS", 4);
    write_int16(ofs, width_); ofs.write("", 2);
    ofs.write("YPTS", 4);
    write_int16(ofs, height_); ofs.write("", 2);

    ofs.write("SCAL", 4);
    ofs.write((char *) &scale_[0], 4);
    ofs.write((char *) &scale_[1], 4);
    ofs.write((char *) &scale_[2], 4);

    ofs.write("CRAD", 4);
    ofs.write((char *) &planet_radius_, 4);

    ofs.write("CRVM", 4);
    write_int32(ofs, render_mode_);

    ofs.write("ALTW", 4);
    write_int16(ofs, height_scale_);
    write_int16(ofs, base_height_);
    for (int i = 0; i < width_*height_; ++i)
    {
        write_int16(ofs, absoluteAltitudeToRelative( altitudes_[i]));
    }

    ofs.close();
    return true;
}

#ifdef TIFF_LIB_EXISTS
bool Terrain::saveTiff(const char *path) const
{
    TIFF *tif = TIFFOpen(path,"w");

    if (!tif)
    {
        std::cerr << "Error opening TIFF image for writing: " << path << "\n" << std::flush;
        return false;
    }

    float alt_range = max_altitude_ - min_altitude_;

    TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, width_);
    TIFFSetField(tif, TIFFTAG_IMAGELENGTH, height_);
    TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
    TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 16);
    TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
    TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_SEPARATE);
    TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
    TIFFSetField(tif, TIFFTAG_XRESOLUTION, 100*scale_[0]);
    TIFFSetField(tif, TIFFTAG_YRESOLUTION, 100*scale_[0]);
    TIFFSetField(tif, TIFFTAG_RESOLUTIONUNIT, RESUNIT_CENTIMETER);
    TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, 0);

    TIFFSetField(tif, kTIFFTAG_TERRAINSCALE, scale_[0]);
    TIFFSetField(tif, kTIFFTAG_TERRAINHEIGHTSCALE, height_scale_);
    TIFFSetField(tif, kTIFFTAG_TERRAINBASEHEIGHT, base_height_);
    TIFFSetField(tif, kTIFFTAG_TERRAINMINALTITUDE, min_altitude_);
    TIFFSetField(tif, kTIFFTAG_TERRAINALTRANGE, alt_range);

    TIFFSetField(tif, kTIFFTAG_TERRAINPLANETRADIUS, planet_radius_);
    TIFFSetField(tif, kTIFFTAG_TERRAINRENDERMODE, render_mode_);

    unsigned short *row = new unsigned short [width_];
    for (short y = 0; y < height_; y++)
    {
        for (short x = 0; x < width_; x++)
        {
            row[x] = (getAltitude(x, y) - min_altitude_)*65355.0/alt_range;
        }
        TIFFWriteScanline(tif, row, y, 0);
    }
    delete [] row;

    TIFFClose(tif);
    return true;
}

bool Terrain::loadTiff(const char *path)
{
    TIFF *tif = TIFFOpen(path,"r");

    if (!tif)
    {
        std::cerr << "Error opening TIFF image: " << path << "\n" << std::flush;
    }

    setDefaultProperties();
    eraseData();
    float alt_range;

    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width_);
    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height_);

    TIFFGetField(tif, kTIFFTAG_TERRAINSCALE, &scale_[0]);
    TIFFGetField(tif, kTIFFTAG_TERRAINHEIGHTSCALE, &height_scale_);
    TIFFGetField(tif, kTIFFTAG_TERRAINBASEHEIGHT, &base_height_);
    TIFFGetField(tif, kTIFFTAG_TERRAINPLANETRADIUS, &planet_radius_);
    TIFFGetField(tif, kTIFFTAG_TERRAINRENDERMODE, &render_mode_);
    TIFFGetField(tif, kTIFFTAG_TERRAINMINALTITUDE, &min_altitude_);
    TIFFGetField(tif, kTIFFTAG_TERRAINALTRANGE, &alt_range);

    scale_[2] = scale_[1] = scale_[0];

    altitudes_ = new float [width_*height_];
    unsigned short *row = new unsigned short [width_];
    for (short y = 0; y < height_; y++)
    {
        TIFFReadScanline(tif, row, y, 0);
        for (short x = 0; x < width_; x++)
        {
            setAltitude(x, y, (row[x]*alt_range/65355.0f) + min_altitude_);
        }
    }
    delete [] row;

    TIFFClose(tif);
    return true;
}

static TIFFExtendProc _ParentExtender = NULL;

static void _XTIFFDefaultDirectory(TIFF *tif)
{
    /* Install the extended Tag field info */
    static const TIFFFieldInfo xtiffFieldInfo[] = {
        {  kTIFFTAG_TERRAINSCALE, 1, 1, TIFF_FLOAT,
           FIELD_CUSTOM, TRUE,	FALSE,	"TerrainScale" },
        {  kTIFFTAG_TERRAINHEIGHTSCALE, 1, 1, TIFF_SSHORT,
           FIELD_CUSTOM, TRUE,	FALSE,	"TerrainHeightScale" },
        {  kTIFFTAG_TERRAINBASEHEIGHT, 1, 1, TIFF_SSHORT,
           FIELD_CUSTOM, TRUE,	FALSE,	"TerrainBaseHeight" },
        {  kTIFFTAG_TERRAINPLANETRADIUS, 1, 1, TIFF_FLOAT,
           FIELD_CUSTOM, TRUE,	FALSE,	"TerrainPlanetRadius" },
        {  kTIFFTAG_TERRAINRENDERMODE, 1, 1, TIFF_LONG,
           FIELD_CUSTOM, TRUE,	FALSE,	"TerrainRenderMode" },
        {  kTIFFTAG_TERRAINMINALTITUDE, 1, 1, TIFF_FLOAT,
           FIELD_CUSTOM, TRUE,	FALSE,	"TerrainMinAltitude" },
        {  kTIFFTAG_TERRAINALTRANGE, 1, 1, TIFF_FLOAT,
           FIELD_CUSTOM, TRUE,	FALSE,	"TerrainAltRange" }
    };

    TIFFMergeFieldInfo( tif, xtiffFieldInfo,
                        sizeof(xtiffFieldInfo) / sizeof(xtiffFieldInfo[0]) );

    /* Since an XTIFF client module may have overridden
     * the default directory method, we call it now to
     * allow it to set up the rest of its own methods.
     */

    if (_ParentExtender)
        (*_ParentExtender)(tif);
}

static void _XTIFFInitialize(void)
{
    static int first_time=1;

    if (! first_time) return; /* Been there. Done that. */
    first_time = 0;

    /* Grab the inherited method and install */
    _ParentExtender = TIFFSetTagExtender(_XTIFFDefaultDirectory);
}

#else

bool Terrain::saveTiff(const char *path) const
{
	std::cout << "TIFF library does not exists. Unable to save Tiff file: " << path << "\n" << std::flush;
	return false;
}

bool Terrain::loadTiff(const char *path)
{
	std::cout << "TIFF library does not exists. Unable to load Tiff file: " << path << "\n" << std::flush;
	return false;
}

#endif

bool rayTerrainIntersect(Terrain *terrain,
                         Eigen::Vector3f start, Eigen::Vector3f ray_dir,
                         float max_dist, std::vector<Eigen::Vector3f> &intersects,
                         float precision)
{
    bool optimized = false; //NOTE: settng optimized to true is buggy

    float delt = precision;
    const float mint = precision;
    const float maxt = max_dist;
    float lh = 0.0f;
    float lz = 0.0f;
    for( float t = mint; t < maxt; t += delt )
    {
        Eigen::Vector3f  p = start + ray_dir*t;

        const float h = terrain->getInterpolatedAltitude(p[0], p[1]);
        if( p[2] < h )
        {
            // interpolate the intersection distance
            float resT;
            if (optimized) resT = t + delt - delt*(lh-lz)/(p[2]-lz-h+lh);
            else resT = t - 0.5*delt;

            intersects.push_back(start + resT*ray_dir);

            //Stop after one intersection
            return true;
        }
        // allow the error to be proportinal to the distance
        if (optimized)
        {
            delt = precision*t;
            lh = h;
            lz = p[2];
        }
    }
    return intersects.size() > 0;
}
