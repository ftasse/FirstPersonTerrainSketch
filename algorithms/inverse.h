#ifndef INVERSE_H
#define INVERSE_H



#include "../geometry/sketch.h"
#include "../geometry/terrain.h"
#include <TwrDynamicMatrix.h>

#include <vector>


class Inverse{
public:
        

        Inverse(int width, int height);
        
        ~Inverse();
        
        void initConstraints(Terrain* terrain, Sketch* sketch);
        void solve();
        TwrDynamicMatrix<float>* getResultArray();
private:
        Terrain* mTerrain;
        
        int mWidth;
        int mHeight;

        
        TwrDynamicMatrix<float>* mDeformation;
        
        std::vector<Eigen::Vector3f> mControlPoints;
        std::vector<Eigen::Vector3f> mDir;
        std::vector<float> mDifference;
        
        

}; // class Inverse


#endif // INVERSE_H
