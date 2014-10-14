#include "deformation.h"
#include "algorithms/solver.h"
#include "algorithms/erosion.h"
#include "algorithms/gauss.h"
#include "algorithms/inverse.h"

#include <float.h>
#include <iostream>

Deformation::Deformation(Terrain *terrain, Sketch *sketch):
    terrain_(terrain), sketch_(sketch)
{

}

double Deformation::update()
{
        // Solve the constraints
        auto solver = new Solver(terrain_->width(), terrain_->height(), 6, 40);
        solver->initConstraints(terrain_, sketch_);
        solver->solve();
        auto result = solver->getResultArray();



//        auto solver = new Inverse(terrain_->width(), terrain_->height());
//        solver->initConstraints(terrain_, sketch_);
//        solver->solve();
//        auto result = solver->getResultArray();
//        
        
        // Smooth the results
//        auto gauss = new Gauss(terrain_->width(), terrain_->height(), 10);
//        gauss->smooth(result);
        
////         Erode the terrain
//        auto erosion = new Erosion(terrain_->width(), terrain_->height(), 1000);
//        erosion->erode(result);

        double error = 0.0;

        // Update the terrain
        for (int x = 0; x < terrain_->width(); ++x)
        {
                for (int y = 0; y < terrain_->height(); ++y)
                {
                    error += pow(result->getAt(x,y), 2);
                    terrain_->setAltitude(x,y, terrain_->getAltitude(x,y) + result->getAt(x,y));
                }
        }
        
        // Update terrain properties
        terrain_->updateMinMax();
        //terrain_->computeGridNormals(); //no need since normals for lighting are computing in shader

        return sqrt(error);
}

double deformTerrainWithConstraints(
    Terrain *terrain,
    const std::vector<CameraInfo> &camera_infos,
    const std::vector<Terrain::SilhouetteSegment> &constraints)
{
  Sketch *tmp = new Sketch();
  float min_diff = FLT_MAX;

  if (constraints.size() > 0)
  {
      for (unsigned int j = 0; j < constraints.size(); ++j)
      {
          tmp->startStroke();
          for (unsigned int k = 0; k < constraints[j].points.size(); ++k)
          {
              Eigen::Vector3f point = constraints[j].points[k];
              tmp->addPolylinePoint(point);
              min_diff = std::min(min_diff, point[2] - terrain->getInterpolatedAltitude(point[0], point[1]));
          }
          tmp->stopStroke();
      }
  }

  for (unsigned int i = 0; i < camera_infos.size(); ++i)
  {
      // We do this to ensure that the terrain height at the camera position stays the same

      CameraInfo camera_info  = camera_infos[i];
      Eigen::Vector3f eye = camera_info.position;
      eye[2] = terrain->getInterpolatedAltitude(eye[0], eye[1]);

      Eigen::Vector3f front = eye + 2*camera_info.direction.normalized();
      front[2] = terrain->getInterpolatedAltitude(front[0], front[1]);
      Eigen::Vector3f behind = eye - 2*camera_info.direction.normalized();
      behind[2] = terrain->getInterpolatedAltitude(behind[0], behind[1]);

      tmp->startStroke();
      tmp->addPolylinePoint(front);
      tmp->addPolylinePoint(eye);
      tmp->addPolylinePoint(behind);
      tmp->stopStroke();

      Eigen::Vector3f right_dir = camera_info.direction.cross(camera_info.up_vector).normalized();
      Eigen::Vector3f right = eye + 2*right_dir;
      right[2] = terrain->getInterpolatedAltitude(right[0], right[1]);
      Eigen::Vector3f left = eye - 2*right_dir;
      left[2] = terrain->getInterpolatedAltitude(left[0], left[1]);

      tmp->startStroke();
      tmp->addPolylinePoint(right);
      tmp->addPolylinePoint(eye);
      tmp->addPolylinePoint(left);
      tmp->stopStroke();
  }

  if (min_diff < 0)
  {
      std::cout << "Minimum dist from terrain: " << std::abs(min_diff) << std::endl << std::flush;
  }

  tmp->setCameraInfo(camera_infos.front());
//  tmp->save("defomation_sketh.vect");

//  terrain->save("tmo_deformed.ter");
//  terrain->load("tmo_deformed.ter");

  double error = 0;
  if (tmp->numStrokes() > 0)
  {
      Deformation *deformation = new Deformation(terrain, tmp);
      error = deformation->update();
      delete deformation;
  }

  return error;
}
