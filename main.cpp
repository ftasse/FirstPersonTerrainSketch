#include <QtGui/QApplication>
#include "gui/mainwindow.h"

#include "geometry/terrain.h"
#include "algorithms/deformation/deformation.h"
#include "algorithms/conversion_to_3d.h"

int main(int argc, char *argv[])
{    
//    Terrain *terrain = new Terrain ();
//    terrain->load("/home/ponjouta/Documents/projets/first-person-terrain-sketching/data/test.ter");

//    Sketch *sketch = new Sketch ();
//    sketch->load("/home/ponjouta/Documents/projets/first-person-terrain-sketching/data/test.vect");

//    Deformation *deformation = new Deformation(terrain, sketch);
//    deformation->update();
//    terrain_->save("result.ter");
//    terrain_->save("result.tif");

//    delete deformation;
//    delete terrain;
//    delete sketch;

//    ConversionTo3D conversion_to_3d("test_sketch.svg");
//    conversion_to_3d.convert();
//    conversion_to_3d.sketch()->save("result.vect");

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
