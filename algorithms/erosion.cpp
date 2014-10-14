#include "erosion.h"
#define CLAMP(a,b,c)  ((a<b)?b:((a>c)?c:a))


Erosion::Erosion(int width, int height, int nbIterations)
: mIterations(nbIterations)
, mWidth(width)
, mHeight(height)
{
        mWater = new TwrDynamicMatrix<float>(width, height);
        mSediments = new TwrDynamicMatrix<float>(width, height);
}

Erosion::~Erosion()
{
}

void Erosion::erode(TwrDynamicMatrix<float>* terrain)
{
        std::cout << "ERODE" << std::endl;
        mTerrain = terrain;
        
        // Clear data
        mWater->clear();
        mSediments->clear();
//        mVelocities->clear();
        
        // Erosion steps
        std::cout << "mIterations" << mIterations << std::endl;
        for (int i = 0; i < mIterations; ++i)
        {
                addSources();
                computeSediments();
                moveWater();
//                computeVelocities();
//                moveSediments();
        }
        std::cout << "...done." << std::endl;
}


void Erosion::addSources()
{
        // Random amount of water for each pixel
        float maxWater = 0.01;
        float minWater = 0.05;
        
//#pragma omp parallel for
        for (int y = 0; y < mHeight; ++y)
        {
                for (int x = 0; x < mWidth; ++x)
                {
                        mWater->setAt(x,y,  mWater->getAt(x,y) + minWater + float(rand()%100000/100000.0) * (maxWater - minWater));
                }
        }
}

void Erosion::computeSediments()
{
        float terrainSolubility = 0.001;
        
#pragma omp parallel for
        for (int y = 0; y < mHeight; ++y)
        {
                for (int x = 0; x < mWidth; ++x)
                {
                        // Compute the sediment as a function of the water quantity
                        // and terrain solubility
                        float sediment = terrainSolubility * mWater->getAt(x,y);
                        sediment = std::min(sediment, mTerrain->getAt(x,y));
//                        std::cout << "mTerrain->getAt(x,y) " << mTerrain->getAt(x,y) << std::endl;
//                        std::cout << "Sediment " << sediment << std::endl;
                        // Add sediment
                        mSediments->setAt(x,y, mSediments->getAt(x,y) + sediment);
                        // Erode the terrain
                        mTerrain->setAt(x,y, mTerrain->getAt(x,y) - sediment);
                }
        }
}

float Erosion::heightAt(int x, int y)
{
        return mTerrain->getAt(x,y) + mSediments->getAt(x,y);
}


void Erosion::moveWater()
{

       // MOVE X
#pragma omp parallel for
        for (int y = 0; y < mHeight; ++y)
        {
                for (int x = 0; x < mWidth - 1; ++x)
                {
                        float wCur = mWater->getAt(x,y);
                        float wNei = mWater->getAt(x+1,y);
                        float diff = (wNei + heightAt(x+1,y)) - (wCur + heightAt(x,y));
                        float dW = CLAMP(diff/2.0, -wCur/2.0, wNei/2.0);
                        
                        
                        mWater->setAt(x,  y, wCur + dW);
                        mWater->setAt(x+1,y, wNei - dW);
                }
        }
        
        // MOVE Y
#pragma omp parallel for
        for (int x = 0; x < mWidth; ++x)
        {
                for (int y = 0; y < mHeight - 1; ++y)
                {
                        float wCur = mWater->getAt(x,y);
                        float wNei = mWater->getAt(x,y+1);
                        float diff = (wNei + heightAt(x,y+1)) - (wCur + heightAt(x,y));
                        float dW = CLAMP(diff/2.0, -wCur/2.0, wNei/2.0);
                        
                        mWater->setAt(x,y,   wCur + dW);
                        mWater->setAt(x,y+1, wNei - dW);
                }
        }
        
}


//void Erosion::moveSediments()
//{
//#pragma omp parallel for
//        for (int y = 0; y < mHeight; ++y)
//        {
//                for (int x = 0; x < mWidth; ++x)
//                {
//                        
//                }
//        }
//}















