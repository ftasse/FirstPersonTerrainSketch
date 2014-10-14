
#include "inverse.h"
#include <Eigen/Eigen>

using namespace std;

Inverse::~Inverse()
{
        delete mDeformation;
}

Inverse::Inverse(int width, int height)
        : mWidth(width)
        , mHeight(height)
{
        mDeformation = new TwrDynamicMatrix<float>(width, height);
}

float norm (const Eigen::Vector3f& a)
{
        return sqrt( (a[0]*a[0])+(a[1]*a[1])+(a[2]*a[2])) ;
}

float dot (const Eigen::Vector3f& a, const Eigen::Vector3f& b)
{
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

Eigen::Vector3f normalize (const Eigen::Vector3f& a)
{
        float n = norm(a);
        return Eigen::Vector3f(a[0]/n, a[1]/n, a[2]/n);
}

void Inverse::initConstraints(Terrain* terrain, Sketch* sketch)
{
        mTerrain = terrain;
        // Init the control points
        for (int i = 0; i < sketch->numStrokes(); ++i)
        {
                Stroke *stroke = sketch->getStroke(i);
                auto points = stroke->getPoints();
                for (int j = 0; j < int(points.size()-1); ++j)
                {
                        Stroke::Point pt0 = points[j];
                        Stroke::Point pt1 = points[j+1];
//                        if (pt0 == pt1)
//                                continue;
                        // Linear interpolation between the stroke points
                        float t = 0;
                        float dt = 0.01;
                        while (t <= 1)
                        {
                                Eigen::Vector3f pt = (1.0-t)*pt0 + t*pt1;
//                                Eigen::Vector3f pt = points[j];
                                
                                float diff = pt[2]- mTerrain->getAltitude(pt[0],pt[1]);
                                mDifference.push_back(diff);
                                
                                pt[2] = mTerrain->getAltitude(pt[0],pt[1]); // set point to terrain
                                mControlPoints.push_back(pt);
                                mDir.push_back(normalize(pt1-pt0));
                                
                                t+= dt;
                        }
                }
        }
        
        for (int y = 0; y < mWidth; ++y)
        {
                for (int x = 0; x < mHeight; ++x)
                {
                        if (x==0 || x == mHeight-1 || y == 0 || y == mWidth-1)
                        {
                                mDifference.push_back(0);
                                mControlPoints.push_back(Eigen::Vector3f(x,y,0));
                                mDir.push_back(Eigen::Vector3f(0,0,0));
                        }
                }
        }
}


void Inverse::solve()
{
#pragma omp parallel for
        for (int y = 0; y < mWidth; ++y)
        {
                float weights;
                float weight;
                float value;
                float n;
                Eigen::Vector3f v;
                Eigen::Vector3f dir;
                float minDist = 999999;
                for (int x = 0; x < mHeight; ++x)
                {
                        weights = 0;
                        value = 0;
                        Eigen::Vector3f pt(x,y,mTerrain->getAltitude(x,y));
                        for (int i = 0; i < mControlPoints.size(); ++i)
                        {
                                v = Eigen::Vector3f(mControlPoints[i]-pt);
//                                v[2] = 0;
                                n = norm(v);
                                weight = 1.0/(n*n);
//                                dir = mDir[i];
//                                if (norm(dir) != 0)
//                                {
//                                        weight *= 1.0-abs(dot(normalize(v),dir));
//                                }
                                value += weight * mDifference[i];
                                weights += weight;
                        }
                        value /= weights;
                        mDeformation->setAt(x,y,value);
                }
        }
}


TwrDynamicMatrix<float>* Inverse::getResultArray()
{
        return mDeformation;
}
