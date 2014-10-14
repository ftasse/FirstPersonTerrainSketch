#ifndef GAUSS_H
#define GAUSS_H



#include "../geometry/sketch.h"
#include "../geometry/terrain.h"
#include <TwrDynamicMatrix.h>

#include <vector>

class Gauss{
public:
        Gauss(int width, int height, int nbIterations);
        ~Gauss();
        void smooth(TwrDynamicMatrix<float>* values);
private:
        int mWidth;
        int mHeight;
        
        TwrDynamicMatrix<float>* pong;
        
        int mIterations;

}; // class Gauss


#endif // GAUSS_H
