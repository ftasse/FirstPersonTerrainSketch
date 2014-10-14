FirstPersonTerrainSketch
========================

Requirements
------------
* Cmake
* QT4
* QGLViewer
* GLEW
* libtiff (http://www.libtiff.org/ or sudo apt-get install libtiff4-dev)
* colorgcc


Compile and Run
---------------
* Clone git repo: git clone git+ssh://username@scm.gforge.inria.fr//gitroot/terrain-sketch/terrain-sketch.git
* cd into terrain-sketch
* Run the compile.sh script
* To run the executable:
    * cd build
    * ./first-person-terrain-sketching
  Useful keystrokes: Ctrl+a to delete intermediate strokes (created for debugging purposes).
                     Ctrl+o to open a terrain file (*.ter)

Organisation
------------

The source code is organised into different folders.

- [ ] algorithms: for all the novel algorithms that we designed for this project. Examples: Deformation

- [ ] tools: for other algorithms we use to in our proposed solution. Examples: SilhouetteExtraction

- [ ] geometry: for all our geometric objects. Examples: Terrain, Sketch, Stroke

- [ ] gui: for all interface-related classes. Examples: TerrainViewer, SketchViewer, MainWindow

- [ ] extern: for external libraries we can bundle with the project.  Examples: Eigen3

- [ ] data: for data we can use for testing such as terrains and sketch files.


References
----------
If you use this code, please refer the related articles:

@inproceedings{Tasse2014a,
 author = {Tasse, Flora Ponjou and Emilien, Arnaud and Cani, Marie-Paule and Hahmann, Stefanie and Bernhardt, Adrien},
 title = {First Person Sketch-based Terrain Editing},
 booktitle = {Proceedings of the 2014 Graphics Interface Conference},
 series = {GI '14},
 year = {2014},
 isbn = {978-1-4822-6003-8},
 location = {Montreal, Quebec, Canada},
 pages = {217--224},
 numpages = {8},
 url = {http://dl.acm.org/citation.cfm?id=2619648.2619684},
 acmid = {2619684},
 publisher = {Canadian Information Processing Society},
 address = {Toronto, Ont., Canada, Canada},
 keywords = {first person editing, silhouettes, sketch-based modelling, terrain},
} 

@article{Tasse2014b,
title = "Feature-based terrain editing from complex sketches ",
journal = "Computers & Graphics ",
volume = "",
number = "0",
pages = " - ",
year = "2014",
note = "",
issn = "0097-8493",
doi = "http://dx.doi.org/10.1016/j.cag.2014.09.001",
url = "http://www.sciencedirect.com/science/article/pii/S0097849314000818",
author = "Flora Ponjou Tasse and Arnaud Emilien and Marie-Paule Cani and Stefanie Hahmann and Neil Dodgson",
keywords = "First person editing",
keywords = "Terrain",
keywords = "Sketch-based modelling",
keywords = "Silhouettes ",
}
