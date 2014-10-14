#include "tools/evaluation.h"

Terrain* filter(Terrain *terrain)
{
  float f_mask[][5] = {	{0.0039, 0.0156, 0.0234, 0.0156, 0.0039},
          {0.0156, 0.0625, 0.0937, 0.0625, 0.0156},
          {0.0234, 0.0937, 0.1406, 0.0937, 0.0234},
          {0.0156, 0.0625, 0.0937, 0.0625, 0.0156},
          {0.0039, 0.0156, 0.0234, 0.0156, 0.0039}
  };

  Terrain* result = new Terrain();
  result->copyFrom(*terrain);
  int i,j,k,l;
  int gauss_width=5;

  for(i=0; i<result->width(); i++)
      {
          for(j=0; j<result->height(); j++)
          {
              float sum=0.0f;
              for(k=0; k<5; k++)
                  for(l=0; l<5; l++)
                  {
                      int x = i-((gauss_width-1)>>1)+k;
                      int y = j-((gauss_width-1)>>1)+l;
                      int rx =  x<0?0:(x>=result->width()?result->width()-1:x);
                      int ry =  y<0?0:(y>=result->height()?result->height()-1:y);

                      sum+=terrain->getRelativeAltitude(rx, ry)*f_mask[k][l];
                  }
              result->setRelativeAltitude(i,j,sum);
          }
      }
  return result;
}

void constructPyramid(Terrain* terrain, std::vector<Terrain*> &pyr, int nlevels)
{
  for (int k = 0; k < pyr.size(); ++k)  delete pyr[k];
  pyr.clear();

  Terrain *cur = new Terrain();
  cur->copyFrom(*terrain);
  pyr.push_back(cur);

  for (int k=1; k<nlevels; k++)
  {
    cur = filter(cur);
    pyr.push_back(cur);
  }

  /*for (int k = 0; k < nlevels; ++k)
    {
      std::stringstream ss; ss << "filtered_" << k << ".ter";
      pyr[k]->save(ss.str().c_str());
    }*/
}

NoiseBandStats* get_noise_stats(std::vector< Terrain* >& levels, int x, int y, int bsize)
{
    int nlevels = ((int) levels.size()) - 1;
    NoiseBandStats* nbs = new NoiseBandStats(nlevels);
    int dimx = bsize, dimy=bsize;

    if (bsize < 0)
      {
        dimx = levels[0]->width();
        dimy = levels[0]->height();
        bsize = dimx;
      }

    float rx = (float) dimx;
    rx *= rx; // diagonal
    float ry = (float) dimy;
    ry *= ry;
    float r = sqrtf(rx + ry);

    float* sigma = new float[nlevels];
    sigma[levels.size()-1] = 1.5f / r; // influence of a single grid element
    for(int i = levels.size()-2; i >= 0; i--)
        sigma[i] = sigma[i+1] * 2.0f;


    unsigned int k;
    int i, j;
    float * diff = new float[dimx*dimy];

    // evaluate
    for(k = 0; k < nlevels; k++)
    {
        // form a difference layer
        // but exclude edges due to boundary issues
        for(i = 0; i < dimx; i++)
            for(j = 0; j < dimy; j++)
            {
                    diff[j+i*dimy] = levels[k+1]->getRelativeAltitude(i+x, j+y) -levels[k]->getRelativeAltitude(i+x, j+y);
            }

        // pass it in for evaluation
        nbs->evalStats(k, (dimx)*(dimy), sigma[k], diff);
    }
    delete [] diff;
    delete [] sigma;

    return nbs;
}

std::vector<float> noise_variances(std::vector<Terrain*> &pyr, int x, int y, int bsize)
{
    std::vector<float> variances;

    NoiseBandStats* nbs =  get_noise_stats(pyr,x,y,bsize);
    nbs->print();

    for(unsigned int i = 0; i <((int)pyr.size())-1; i++)
        variances.push_back((nbs->getStdDev(i))*nbs->getStdDev(i));
    delete nbs;
    return variances;
}

void noise_variances(std::vector<Terrain*> &pyr, float **noise_var_patches, int bsize)
{
  if (!noise_var_patches) return;

  int nlevels = ((int)pyr.size())-1;
  int swidth = pyr.front()->width() / bsize;
  int sheight = pyr.front()->height() / bsize;

  for (int i = 0; i < swidth; ++i)
    for (int j = 0; j < sheight; ++j)
      {
        NoiseBandStats* nbs =  get_noise_stats(pyr,i*bsize,j*bsize,bsize);
        for(unsigned int k = 0; k <nlevels; k++)
            noise_var_patches[k][j*swidth + i] = ((nbs->getStdDev(k))*nbs->getStdDev(k));
        delete nbs;
      }
}
