#ifndef EVALUATION_H
#define EVALUATION_H

#include "geometry/terrain.h"
#include "extern/noise/noise.h"

void constructPyramid(Terrain* terrain, std::vector<Terrain*> &pyr, int nlevels);

NoiseBandStats* get_noise_stats(std::vector< Terrain* >& levels, int x = 0, int y = 0, int bsize = -1);

std::vector<float> noise_variances(std::vector<Terrain *>& pyr, int x = 0, int y = 0, int bsize = -1);

void noise_variances(std::vector<Terrain*> &pyr, float **noise_var_patches, int bsize);
#endif // EVALUATION_H
