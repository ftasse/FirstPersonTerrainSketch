#ifndef SOLVER_H
#define SOLVER_H



#include "../geometry/sketch.h"
#include "../geometry/terrain.h"
#include <TwrDynamicMatrix.h>

#include <vector>


struct Grad{
        float dx;
        float dy;
        float norm;
        Grad() : dx(0), dy(0), norm(0) {}
        Grad(float x, float y, float z) : dx(x), dy(y), norm(z) {}
};

Grad operator+(const Grad& a, const Grad& b);
Grad operator*(const float a, const Grad& b);

        
class Solver{
public:
        

        Solver(int width, int height, int nbLOD, int nbIterations);
        ~Solver();
        void initConstraints(Terrain* terrain, Sketch* sketch);
        void solve();
        TwrDynamicMatrix<float>* getResultArray();
private:
        Terrain* mTerrain;
        
        int mWidth;
        int mHeight;

        std::vector<TwrDynamicMatrix<float>*> mConstraints;  ///< Constraints arrays for the diffusion resolution
        std::vector<TwrDynamicMatrix<float>*> mPings;  ///< Temp buffers for the diffusion resolution
        std::vector<TwrDynamicMatrix<float>*> mPongs;  ///< Temp buffers for the diffusion resolution
        
        
        std::vector<TwrDynamicMatrix<Grad>*> mGradConstraints;  ///< Constraints arrays for the diffusion resolution
        std::vector<TwrDynamicMatrix<Grad>*> mGradPings;  ///< Temp buffers for the diffusion resolution
        std::vector<TwrDynamicMatrix<Grad>*> mGradPongs;  ///< Temp buffers for the diffusion resolution

        /** Compute the high level constraints array
        */
        void initHighLevelConstraints(Sketch* sketch);

        /** Compute all the constraints levels from the hight level constraints array
        */
        void updateConstraintsMipmap();
        void mipmapConstraints(TwrDynamicMatrix<float>* high, TwrDynamicMatrix<float>* coarse);
        void mipmapGradConstraints(TwrDynamicMatrix<Grad>* high, TwrDynamicMatrix<Grad>* coarse);

        /** Solve one level of the diffusion process
        */
        void solveLevel(int i);
        void solveGradLevel(int i);

        int mL; ///< Number of LOD level in the resolution
        int mIterations;

}; // class Solver


#endif // SOLVER_H
