
#include "gauss.h"

using namespace std;

Gauss::~Gauss()
{
        delete pong;
}

Gauss::Gauss(int width, int height, int nbIterations)
        : mWidth(width)
        , mHeight(height)
        , mIterations(nbIterations)
{
        pong = new TwrDynamicMatrix<float>(mWidth, mHeight);
}

void Gauss::smooth(TwrDynamicMatrix<float>* values)
{
        auto ping = values;
        
        for (int i = 0; i < mIterations; ++i)
        {
#pragma omp parallel for
                for (int y = 1; y < mHeight-1; ++y)
                {
                        for (int x = 1; x < mWidth - 1; ++x)
                        {
                                float val =     (ping->getAt(x-1, y-1) 
                                              +  ping->getAt(x+1, y+1) 
                                              +  ping->getAt(x-1, y+1) 
                                              +  ping->getAt(x+1, y-1) ) / 16.f
                                          +     (ping->getAt(x-1, y) 
                                              +  ping->getAt(x+1, y) 
                                              +  ping->getAt(x, y-1) 
                                              +  ping->getAt(x, y+1) ) / 8.f
                                          +     ping->getAt(x, y) / 4.f;
                                pong->setAt(x,y,val);
                        }
                }
                auto swap = ping;
                ping = pong;
                pong = swap;
        }
}


