cmake_minimum_required(VERSION 2.8)
project (first-person-terrain-sketching)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()

add_definitions(-DNOMINMAX)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(OpenGL REQUIRED)
# message(STATUS "OPENGL_LIBRARIES=${OPENGL_LIBRARIES}")

find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_PATH})

find_package(TIFF)
if (TIFF_FOUND)
	add_definitions(-DTIFF_LIB_EXISTS)
endif()

find_package(Qt4 REQUIRED)
set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTXML TRUE)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

find_package(QGLViewer REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

include_directories(extern/)
include_directories(extern/eigen3)
include_directories(extern/Twr/)

set(HEADERS_FILES 
            extern/noise/noise.h

            tools/evaluation.h

            tools/math_extras.h
            tools/silhouette_extractor.h
	    tools/camera_info.h 
	    tools/offline_renderer.h 
	    tools/offline_terrain_renderer.h 
	    tools/shader_utils.h 

	    tools/ppa/headers.h 
	    tools/feature_detection.h 
	    tools/ppa/dtts_ppa.h 
	    tools/ppa/dtts_mst.h
	    tools/deformation_weighting.h 

            geometry/terrain.h
	    geometry/stroke.h 
	    geometry/sketch.h 
	    geometry/polyline.h 
	    geometry/spline.h 

	    gui/terrain_fast_renderer.h 

	    algorithms/solver.h 
	    algorithms/gauss.h 
	    algorithms/erosion.h  
	    algorithms/inverse.h
	    algorithms/conversion_to_3d.h 
	    algorithms/polyline_completion.h 
	    algorithms/deformation/deformation.h
            algorithms/deformation/feature_matching.h
            algorithms/deformation/extended_feature_matching.h
            algorithms/deformation/protruded_silhouettes_matching.h
)

set(SRC_FILES 
        main.cpp

        extern/noise/noise.cpp
        tools/evaluation.cpp


        tools/feature_detection.cpp
        tools/ppa/dtts_ppa.cpp
        tools/camera_info.cpp
        tools/offline_renderer.cpp
        tools/offline_terrain_renderer.cpp
        tools/silhouette_extractor.cpp

        gui/mainwindow.cpp 
    	gui/terrain_viewer.cpp 
    	gui/sketch_viewer.cpp 
    	gui/terrain_fast_renderer.cpp 

    	geometry/terrain.cpp 
    	geometry/stroke.cpp 
    	geometry/sketch.cpp
    	geometry/polyline.cpp 
    	geometry/spline.cpp

    	algorithms/solver.cpp 
    	algorithms/erosion.cpp 
	    algorithms/inverse.cpp 
	    algorithms/gauss.cpp     	     
	    algorithms/conversion_to_3d.cpp
	    algorithms/polyline_completion.cpp 
	    algorithms/deformation/deformation.cpp
            algorithms/deformation/feature_matching.cpp
            algorithms/deformation/extended_feature_matching.cpp
            algorithms/deformation/protruded_silhouettes_matching.cpp
)

file(GLOB_RECURSE SHADER_FILES
        shaders/*.glsl
)

# file(GLOB_RECURSE TEXTURE_FILES 
# 	data/terrain_textures/*.jpg
# )

SET(QT_FORMS_FILES mainwindow.ui)
set(QT_HEADERS_FILES 
		gui/mainwindow.h
	    gui/terrain_viewer.h 
		gui/sketch_viewer.h 
)

QT4_WRAP_CPP(QT_HEADERS_MOC ${QT_HEADERS_FILES})
QT4_WRAP_UI(QT_FORMS_HEADERS ${QT_FORMS_FILES})

add_executable (first-person-terrain-sketching 
	${QT_HEADERS_MOC} ${QT_FORMS_HEADERS} 
	${HEADERS_FILES} ${SRC_FILES}
        ${SHADER_FILES}
) 
target_link_libraries (first-person-terrain-sketching 
	${TIFF_LIBRARIES}
	${QT_LIBRARIES}
	${QGLVIEWER_LIBRARY}
	${OPENGL_LIBRARIES} ${GLEW_LIBRARY}
)

# message(STATUS "QT4_LIBRARIES = ${QT_LIBRARIES}")


# add_custom_command(TARGET first-person-terrain-sketching POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy_directory
#         ${CMAKE_SOURCE_DIR}/data/terrain_textures
#         $<TARGET_FILE_DIR:first-person-terrain-sketching>/terrain_textures)

add_custom_target(copy_files ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/shaders
        $<TARGET_FILE_DIR:first-person-terrain-sketching>/shaders
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/data/terrain_textures
        $<TARGET_FILE_DIR:first-person-terrain-sketching>/terrain_textures
)

# file(COPY ${CMAKE_SOURCE_DIR}/shaders DESTINATION ${CMAKE_BINARY_DIR})
# file(COPY ${CMAKE_SOURCE_DIR}/data/terrain_textures DESTINATION ${CMAKE_BINARY_DIR})
