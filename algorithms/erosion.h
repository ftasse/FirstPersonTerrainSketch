#ifndef EROSION_H
#define EROSION_H


#include "../geometry/sketch.h"
#include "../geometry/terrain.h"
#include <TwrDynamicMatrix.h>
#include <Eigen/Geometry>

#include <vector>

class Erosion{
public:
        Erosion(int width, int height, int nbIterations);
        ~Erosion();
        void erode(TwrDynamicMatrix<float>* terrain);
private:
        int mWidth;
        int mHeight;
        int mIterations;

        TwrDynamicMatrix<float>* mTerrain;
        TwrDynamicMatrix<float>* mWater;
        TwrDynamicMatrix<float>* mWaterBack;
        TwrDynamicMatrix<float>* mSediments;
        
        float heightAt(int x, int y);

        void addSources();
        void computeSediments();
        void moveWater();
//        void computeVelocities();
//        void moveSediments();


}; // class Erosion


#endif // EROSION_H
