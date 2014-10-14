#ifndef DEFORMATION_WEIGHTING_H
#define DEFORMATION_WEIGHTING_H

#include <string>

#define kSimilarityWeight 0
#define kDeformationWeight 1
#define kSamplingWeight 2
#define kFeatureExtensionWeight 3

const static std::string weight_titles[4] = {"similarity weight:", "deformation weight:", "feature sampling weight:",
                                             "feature extrusion weight:"};

struct DeformationWeighting
{
    float weights[5];

    DeformationWeighting()
    {
        weights[kSimilarityWeight] = 1.0; //0.8;
        weights[kDeformationWeight] = 1.0; //0.8;
        weights[kSamplingWeight] = 1.0; //0.100;
        weights[kFeatureExtensionWeight] = 1.0; //0.5000;
    }

    std::string getWeightTitle(int id)
    {
        return weight_titles[id];
    }

    int numWeights()
    {
        return 4;
    }
};



#endif // DEFORMATION_WEIGHTING_H
