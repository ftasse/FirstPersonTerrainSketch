
#include "solver.h"
// #include "../extern/Twr/TwrTimer.hpp"
// using namespace Tower;

using namespace std;

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

float dot (const Eigen::Vector2f& a, const Eigen::Vector2f& b)
{
        return a[0]*b[0] + a[1]*b[1];
}

float norm (const Eigen::Vector2f& a)
{
        return sqrt( (a[0]*a[0])+(a[1]*a[1])) ;
}

Eigen::Vector2f normalize (const Eigen::Vector2f& a)
{
        float n = norm(a);
        return Eigen::Vector2f(a[0]/n, a[1]/n);
}


Grad operator+(const Grad& a, const Grad& b) {
        return Grad(a.dx+b.dx, a.dy+b.dy, a.norm+b.norm);
}
Grad operator*(const float a, const Grad& b) {
        return Grad(a*b.dx, a*b.dy, a*b.norm);
}

Solver::~Solver()
{
        mConstraints.clear();
        mPings.clear();
        mPongs.clear();
}

Solver::Solver(int width, int height, int nbLOD, int nbIterations)
        : mWidth(width)
        , mHeight(height)
        , mL(nbLOD)
        , mIterations(nbIterations)
{
        // Init arrays
        for (int i = 0; i < mL; ++i)
        {
                int width  = mWidth  / pow(2.0f,i);
                int height = mHeight / pow(2.0f,i);
                mConstraints.push_back( new TwrDynamicMatrix<float>(width, height) );
                mPings.push_back( new TwrDynamicMatrix<float>(width, height) );
                mPongs.push_back( new TwrDynamicMatrix<float>(width, height) );

                mGradConstraints.push_back( new TwrDynamicMatrix<Grad>(width, height) );
                mGradPings.push_back( new TwrDynamicMatrix<Grad>(width, height) );
                mGradPongs.push_back( new TwrDynamicMatrix<Grad>(width, height) );
        }
        for (int i = 0; i < mL; ++i)
        {
                mConstraints[i]->clear();
                mPings[i]->clear();
                mPongs[i]->clear();

                mGradConstraints[i]->clear();
                mGradPings[i]->clear();
                mGradPongs[i]->clear();
        }
}

void Solver::initConstraints(Terrain* terrain, Sketch* sketch)
{
        mTerrain = terrain;
        if (sketch != NULL)
        {
                initHighLevelConstraints(sketch);
                updateConstraintsMipmap();
        }
}

void Solver::initHighLevelConstraints(Sketch* sketch)
{
        // Clear the constraints
        int width  = mConstraints[0]->getNbColumns();
        int height = mConstraints[0]->getNbLines();

        mConstraints[0]->clear();


        for (int y = 0; y < height; ++y)
        {
                for (int x = 0; x < width; ++x)
                {
                        mConstraints[0]->setAt(x,y, -999999);
                }
        }


        mGradConstraints[0]->clear();
        // For each stroke
        for (int i = 0; i < sketch->numStrokes(); ++i)
        {
                Stroke *stroke = sketch->getStroke(i);
                auto points = stroke->getPoints();
                for (int j = 0; j < int(points.size()-1); ++j)
                {
                        Stroke::Point pt0 = points[j];
                        Stroke::Point pt1 = points[j+1];
                        // Compute stroke direction
                        float dirx = pt1[0] -  pt0[0];
                        float diry = pt1[1] -  pt0[1];
                        float dirn = sqrt(dirx*dirx + diry*diry);
                        dirx/=dirn;
                        diry/=dirn;
                        // Compute storke normal
                        float normalx = diry;
                        float normaly = - dirx;
                        // Linear interpolation between the stroke points
                        float t = 0;
                        float dt = 0.001;
                        while (t <= 1)
                        {
                                Stroke::Point pt = (1.0-t)*pt0 + t*pt1;
                                int x = pt[0];
                                int y = pt[1];
                                float z = pt[2];
                                // Height difference constraint at x,y
                                z = z - mTerrain->getAltitude(x,y);
                                mConstraints[0]->setAt(x,y,z);
                                mGradConstraints[0]->setAt( x, y, Grad(0,0, 1));

                                // Write gradient constraint along the curve
                                if (dirn > 0.001)
                                {
                                        int gradRadius = 16;


                                        for (float r = -gradRadius; r <= gradRadius; r+=3.0)
                                        {

                                                int xx = int(float(x) + normalx * r);
                                                int yy = int(float(y) + normaly * r);
                                                if (xx >= 0 && xx < width && yy >= 0 && yy < height)
                                                {
//                                                        mGradConstraints[0]->setAt( xx, yy, Grad(normalx*sign(r), normaly*sign(r), 0));
//                                                        mConstraints[0]->setAt(xx,yy,z);
                                                        mGradConstraints[0]->setAt(xx, yy, Grad(normalx*sign(r), normaly*sign(r), float(gradRadius-r)/float(gradRadius)));
                                                        if (mGradConstraints[0]->getAt(xx, yy).norm != 1) {
                                                                mGradConstraints[0]->setAt( xx, yy, Grad(normalx*sign(r), normaly*sign(r), 1));
                                                        }

                                                        if (j == 0|| j == int(points.size()-2))
                                                        {
                                                                mConstraints[0]->setAt(xx,yy,z);
                                                                mGradConstraints[0]->setAt( xx, yy, Grad(normalx*sign(r), normaly*sign(r), 2));
                                                        }
                                                }
                                        }


//                                        for (float r = gradRadius; r > 0; r-=0.25)
//                                        {
//                                                for (int m = -gradRadius; m <= gradRadius; ++m){
//                                                        for (int n = -gradRadius; n <= gradRadius; ++n){
//
//                                                                if (sqrt(m*m+n*n) > gradRadius)
//                                                                        continue;
//
//                                                                int xx = int(float(x) + m);
//                                                                int yy = int(float(y) + n);
////                                                                int xx = int(float(x) + normalx * r);
////                                                                int yy = int(float(y) + normaly * r);
//                                                                if (xx >= 0 && xx < width && yy >= 0 && yy < height)
//                                                                {
//                //                                                        mGradConstraints[0]->setAt( xx, yy, Grad(normalx*sign(r), normaly*sign(r), 0));
//                                                                        mConstraints[0]->setAt(xx,yy,z);
//                                                                        mGradConstraints[0]->setAt(xx, yy, Grad(normalx*sign(r), normaly*sign(r), float(gradRadius-r)/float(gradRadius)));
//                                                                        if (mGradConstraints[0]->getAt(xx, yy).norm != 1) {
//                                                                                mGradConstraints[0]->setAt( xx, yy, Grad(normalx*sign(r), normaly*sign(r), 1));
//                                                                        }
//
//                //                                                        if (j == 0|| j == int(points.size()-2))
//                //                                                        {
//                //                                                                mConstraints[0]->setAt(xx,yy,z);
//                //                                                                mGradConstraints[0]->setAt( xx, yy, Grad(normalx*sign(r), normaly*sign(r), 2));
//                //                                                        }
//                                                                }
//
//                                                        }
//                                                }
//                                        }
                                }

                                // Iterate interpolation
                                t+=dt;
                        }
                }

//                {
//                        Stroke::Point pt0 = points[0];
//                        Stroke::Point pt1 = points[0+5];
//
//                        float dirx = pt1[0] -  pt0[0];
//                        float diry = pt1[1] -  pt0[1];
//                        float dirn = sqrt(dirx*dirx + diry*diry);
//                        dirx/=dirn;
//                        diry/=dirn;
//                        float normalx = diry;
//                        float normaly = - dirx;
//
//                        int x = pt0[0];
//                        int y = pt0[1];
//                        float z = pt0[2];
//
//                        int gradRadius = 30;
//                        for (float r = -gradRadius; r <= gradRadius; r+=0.25)
//                        {
//                                int xx = int(float(x) - normalx * r);
//                                int yy = int(float(y) - normaly * r);
//                                mConstraints[0]->setAt(xx,yy,z);
//                                mGradConstraints[0]->setAt(xx, yy, Grad(normalx*sign(r), normaly*sign(r), float(gradRadius-r)/float(gradRadius)));
//                        }
//                }

//                {
//                        Stroke::Point pt0 = points[points.size()-2];
//                        Stroke::Point pt1 = points.back();
//
//                        float dirx = pt1[0] -  pt0[0];
//                        float diry = pt1[1] -  pt0[1];
//                        float dirn = sqrt(dirx*dirx + diry*diry);
//                        dirx/=dirn;
//                        diry/=dirn;
//
//                        mGradConstraints[0]->setAt( pt0[0], pt0[1], Grad(+dirx,+diry,1));
//                }
        }
}

void Solver::updateConstraintsMipmap()
{
        for (int i = 0; i < mL - 1; i++)
        {
                auto highConstraints = mConstraints[i];
                auto constraints = mConstraints[i+1];
                mipmapConstraints(highConstraints, constraints);

//                auto highGradConstraints = mGradConstraints[i];
//                auto gradConstraints = mGradConstraints[i+1];
//                mipmapGradConstraints(highGradConstraints, gradConstraints);
        }
}




void Solver::mipmapGradConstraints(TwrDynamicMatrix<Grad>* high, TwrDynamicMatrix<Grad>* coarse)
{
        // Set to Zero
        coarse->clear();

        // Mipmap
#pragma omp parallel for
        for (int y = 0; y < int(coarse->getNbLines()-1); ++y)
        {
                for (int x = 0; x < int(coarse->getNbColumns()-1); ++x)
                {
                        // Not exactly a mipmap,
                        // Because we take into account only the non-zero constraints
                        Grad val00 = high->getAt(x*2,y*2);
                        Grad val01 = high->getAt(x*2,y*2+1);
                        Grad val10 = high->getAt(x*2+1,y*2);
                        Grad val11 = high->getAt(x*2+1,y*2+1);
                        Grad val;
                        int nbVal = 0;
                        if (val00.dx!= 0 || val00.dy != 0) {
                                val = val + val00;
                                nbVal++;
                        }
                        if (val01.dx!= 0 || val01.dy != 0) {
                                val = val + val01;
                                nbVal++;
                        }
                        if (val10.dx!= 0 || val10.dy != 0) {
                                val = val + val10;
                                nbVal++;
                        }
                        if (val11.dx!= 0 || val11.dy != 0) {
                                val = val + val11;
                                nbVal++;
                        }
                        if (nbVal > 0) {
                                float norm = sqrt(val.dx*val.dx + val.dy*val.dy);
//                                if (norm > 0)
//                                {
                                        val.dx /= float(norm);
                                        val.dy /= float(norm);
//                                }
//                                else
//                                {
//                                        val.dx = 0;
//                                        val.dy = 0;
//                                }
                                val.norm /= float(nbVal);
                                coarse->setAt(x,y,val);
                        }
                }
        }
}


void Solver::mipmapConstraints(TwrDynamicMatrix<float>* high, TwrDynamicMatrix<float>* coarse)
{
        // Set to Zero
        coarse->clear();

        for (int y = 0; y < coarse->getNbLines(); ++y)
        {
                for (int x = 0; x < coarse->getNbColumns(); ++x)
                {
                        coarse->setAt(x,y, -999999);
                }
        }

        // Mipmap
#pragma omp parallel for
        for (int y = 0; y < int(coarse->getNbLines()-1); ++y)
        {
                for (int x = 0; x < int(coarse->getNbColumns()-1); ++x)
                {
                        // Not exactly a mipmap,
                        // Because we take into account only the non-zero constraints
                        float val00 = high->getAt(x*2,y*2);
                        float val01 = high->getAt(x*2,y*2+1);
                        float val10 = high->getAt(x*2+1,y*2);
                        float val11 = high->getAt(x*2+1,y*2+1);
                        float val = 0;
                        int nbVal = 0;
                        if (val00 != -999999) {
                                val+=val00;
                                nbVal++;
                        }
                        if (val01 != -999999) {
                                val+=val01;
                                nbVal++;
                        }
                        if (val10 != -999999) {
                                val+=val10;
                                nbVal++;
                        }
                        if (val11 != -999999) {
                                val+=val11;
                                nbVal++;
                        }
                        if (nbVal > 0) {
                                coarse->setAt(x,y,val/float(nbVal));
                        }
                }
        }
}



void Solver::solveGradLevel(int stepnb)
{
        auto ping =  mGradPings[stepnb];
        auto pong =  mGradPongs[stepnb];
        auto constraints =  mGradConstraints[stepnb];

        int width  = ping->getNbColumns();
        int height = ping->getNbLines();

        // Init ping
        // If first step to zero
        if (stepnb == mL-1)
        {
                ping->clear();
        }
        // Else to previous step values
        else
        {
                auto previousPing = mGradPings[stepnb+1];
#pragma omp parallel for
                for (int y = 0; y < height; ++y)
                {
                        for (int x = 0; x < width; ++x)
                        {
                                ping->setAt(x,y, previousPing->getAt(x/2,y/2));
                        }
                }
        }

        // Iterations (ping-pong)
        for (int k = 0; k < mIterations; ++k)
        {
#pragma omp parallel for
                for (int y = 1; y < height-1; ++y)
                {
                        for (int x = 1; x < width-1; ++x)
                        {
                                // If a constraint exist, the pong value takes it
                                Grad constraint = constraints->getAt(x,y);
                                if (constraint.dx != 0 || constraint.dy != 0 || constraint.norm != 0)
                                {
                                        pong->setAt(x,y,constraint);
                                }
                                // Else, interpolation of neightbors
                                else
                                {
                                        Grad valm10 = ping->getAt(x-1,y);
                                        Grad val0m1 = ping->getAt(x,y-1);
                                        Grad val01 = ping->getAt(x,y+1);
                                        Grad val10 = ping->getAt(x+1,y);

                                        Grad val = 0.25* (valm10 + val0m1 + val10 + val01);
                                        // Laplacian
                                        pong->setAt(x,y,val);
                                }
                        }
                }

                auto swap = ping;
                ping  = pong;
                pong  = swap;
        }
}

void Solver::solveLevel(int stepnb)
{
        auto ping =  mPings[stepnb];
        auto pong =  mPongs[stepnb];
        auto constraints =  mConstraints[stepnb];
        auto grads =  mGradPings[0];
        auto gradCs =  mGradConstraints[stepnb];

        int width  = ping->getNbColumns();
        int height = ping->getNbLines();

        // Init ping
        // If first step to zero
        if (stepnb == mL-1)
        {
                ping->clear();
                for (int y = 1; y < height-1; ++y)
                {
                        for (int x = 1; x < width-1; ++x)
                        {
                                ping->setAt(x,y, -10);
                        }
                }
        }
        // Else to previous step values
        else
        {
                auto previousPing = mPings[stepnb+1];
#pragma omp parallel for
                for (int y = 0; y < height; ++y)
                {
                        for (int x = 0; x < width; ++x)
                        {
                                ping->setAt(x,y, previousPing->getAt(x/2,y/2));
                        }
                }
        }

        // Iterations (ping-pong)
        for (int k = 0; k < mIterations; ++k)
        {
#pragma omp parallel for
                for (int y = 1; y < height-1; ++y)
                {
                
                        float id;

                        // Laplacian
                        float valm10;
                        float val0m1;
                        float val01;
                        float val10;
                        
                        float laplace;


                        for (int x = 1; x < width-1; ++x)
                        {
                                // Identification
                                id = constraints->getAt(x,y);

                                // Laplacian
                                valm10 = ping->getAt(x-1,y);
                                val0m1 = ping->getAt(x,y-1);
                                val01 = ping->getAt(x,y+1);
                                val10 = ping->getAt(x+1,y);


                                laplace = 0.25 * (valm10 + val0m1 + val10 + val01);


                                if (id != -999999)
                                {

                                        pong->setAt(x,y, id);
                                }
                                else
                                {
                                        pong->setAt(x,y, laplace) ;
                                }
                        }
                }

                auto swap = ping;
                ping  = pong;
                pong  = swap;
        }
//
//        if (stepnb != 0)
        {
        for (int k = 0; k < 6; ++k)
        {
#pragma omp parallel for
                for (int y = 1; y < height-1; ++y)
                {
                        for (int x = 1; x < width-1; ++x)
                        {
                                pong->setAt(x,y,
                                                 1.0/16.0*(
                                                         ping->getAt(x-1,y-1)
                                                         + ping->getAt(x-1,y+1)
                                                         + ping->getAt(x+1,y+1)
                                                         + ping->getAt(x+1,y-1))
                                                + 1.0/8.0*(
                                                         ping->getAt(x-1,y)
                                                         + ping->getAt(x,y+1)
                                                         + ping->getAt(x+1,y)
                                                         + ping->getAt(x,y-1))
                                                 + 1.0/4.0*ping->getAt(x,y) );
                        }
                }
                auto swap = ping;
                ping  = pong;
                pong  = swap;
        }
        }
}



void Solver::solve()
{
        // TwrTimer a;
        // a.tick();
        // Solve

//        // Grad From coarse to fine
//        std::cout << "SOLVE Grad" << std::endl;
//        for (int i = mL - 1; i >= 0; --i)
//        {
//                solveGradLevel(i);
//        }

        // Height From coarse to fine
        std::cout << "SOLVE Height" << std::endl;
        for (int i = mL - 1; i >= 0; --i)
        {
                solveLevel(i);
        }


    // std::cout << "Solver: " << a.getDtInSec() << " secs" << std::endl << std::flush;
}


TwrDynamicMatrix<float>* Solver::getResultArray()
{
        if (mIterations%2 == 0)
        {
                return mPings[0];
        }
        else
        {
                return mPongs[0];
        }
}




