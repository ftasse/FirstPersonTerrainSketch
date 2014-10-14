#ifndef CONVERSION_TO_3D_H
#define CONVERSION_TO_3D_H

#include "geometry/sketch.h"

#include <map>

class ConversionTo3D
{
    struct SilSegment   //Silhouette segment (represents projection on XY plane)
    {
        Eigen::Vector3f from_point, to_point;
        int stroke_id;
        float max_alt;

        SilSegment(Eigen::Vector3f _from_point = Eigen::Vector3f(0, 0, 0),
                   Eigen::Vector3f _to_point = Eigen::Vector3f(0, 0, 0)):
            from_point(_from_point),
            to_point(_to_point) {}

        bool operator <(const SilSegment &rhs) const
        {
            return max_alt < rhs.max_alt;
        }
    };

    enum SilEndPointLabelType { kRelLiesOn,
                                kRelOccludedBy,
                                kRelTripleJunction,
                                kRelNone};

    struct SilEndPointLabel   //Silhouette endpoint label (such as "lies on stroke 0")
    {
        int other_stroke_id;
        SilEndPointLabelType ltype;

        SilEndPointLabel(int _other_stroke_id = -1,
                         SilEndPointLabelType _ltype = kRelNone):
            other_stroke_id(_other_stroke_id),
            ltype(_ltype){}
    };

    struct SilEndpoint
    {
        int stroke_id;
        int stroke_point_id;
        Eigen::Vector3f pos;
        SilEndPointLabel label;

        int tjunction_stroke_id;

        SilEndpoint() { tjunction_stroke_id = -1; }

        bool operator <(const SilEndpoint &rhs) const
        {
            return pos[0] < rhs.pos[0];
        }
    };

    typedef std::pair<SilEndpoint, SilEndpoint> SilEndpointPair;

    struct Paraboloid //Represented implicitly by z = p0*x^2 + p1*x*y + p2*y^2 + p3*x + p4*y + p5*z
    {
        Eigen::VectorXf params;
    };

public:
    explicit ConversionTo3D(Sketch *sketch);
    explicit ConversionTo3D(const char *sketch_svg_path,
                            Eigen::Vector2i terrain_size = Eigen::Vector2i(512, 512));

    void convert();
    void sortStrokes();
    Sketch *sketch() const { return sketch_; }

private:
    Sketch *sketch_;
    std::vector<int> sorted_stroke_ids_;
    std::vector<float> silhouette_max_alts_;
    std::vector<SilEndpointPair> silhouette_endpoint_pairs_;

    void createSilhouetteEndpointPairs();
    void computeXaxisSweeping();
    void applyFarawayProjection();
    void applyNearerAdjustment();
    Eigen::Vector3f projectOnSketchPlane(Eigen::Vector3f point);

    bool isHiddenEndpoint(const SilEndpoint &endpoint);
    bool aboveStroke(const SilEndpoint &endpoint1,
                     const SilEndpoint &endpoint2);

    void projectSilhouetteOnSilhouette(Stroke* stroke, Stroke* on_stroke,
                                       std::vector<Eigen::Vector3f> &projected);
    void projectSilhouetteOnParaboloid(Stroke* stroke, Paraboloid on_paraboloid,
                                     std::vector<Eigen::Vector3f> &projected);
    void projectSilhouetteOnFlatTerrain(Stroke *stroke,
                                        std::vector<Eigen::Vector3f> &projected);

    Paraboloid getParaboloidForStroke(Stroke *stroke);
};

#endif // CONVERSION_TO_3D_H
