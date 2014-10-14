#include "silhouette_extractor.h"

#include <stdio.h>
#include <iostream>
#include <float.h>
#include <ctime>


#include <Eigen/Geometry>

//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"

#include "geometry/polyline.h"

SilhouetteExtractor::SilhouetteExtractor(Terrain *terrain)
    : OfflineTerrainRenderer(terrain),
      front_facing_(NULL), front_facing_per_tri_(NULL)
{
    assert (terrain_->hasGridNormals());

    int iCPU = omp_get_num_procs();  // Get the number of processors in this system
    omp_set_num_threads(iCPU);
}

SilhouetteExtractor::~SilhouetteExtractor()
{
    delete front_facing_;
    front_facing_ = NULL;
}

/*
void SilhouetteExtractor::computeFromContour(std::vector<Terrain::SilhouetteSegment> &silhouette_segments,
                                             cv::Mat *img)
{
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(*img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    cv::Mat drawing = cv::Mat::zeros( img->size(), CV_8UC3 );
    cv::RNG rng(12345);
    for( unsigned int i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        cv::drawContours( drawing, contours, i, color);
    }
    cv::imwrite("test.png", drawing);

    pixelbuffer_->makeCurrent();
    //if (!use_depth_map)
    {
        glDisable(GL_POLYGON_OFFSET_FILL);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glColor3f(0.0f, 0.0f, 1.0f);
        drawTerrain();
        //saveBuffer("test2.png", GL_RGB);
    }

    updateMatrices();

    for (unsigned int i=0; i < contours.size(); ++i)
    {
        Terrain::SilhouetteSegment segment;
        for (unsigned int j=0; j < contours[i].size(); ++j)
        {
            cv::Point pixel =  contours[i][j];
            Eigen::Vector2f normal(0, 2);
            //            if (j == 0)
            //                normal =  Eigen::Vector2f(contours[i][j+1].x, contours[i][j+1].y) -
            //                          Eigen::Vector2f(pixel.x, pixel.y);
            //            else if (j == contours[i].size()-1)
            //                normal =  Eigen::Vector2f(pixel.x, contours[i][j].y) -
            //                          Eigen::Vector2f(contours[i][j-1].x, contours[i][j-1].y);
            //            else
            //                normal =  Eigen::Vector2f(contours[i][j+1].x, contours[i][j+1].y) -
            //                          Eigen::Vector2f(contours[i][j-1].x, contours[i][j-1].y);
            //            normal = Eigen::Vector2f(-normal[1], normal[0]).normalized();

            bool is_background;
            Eigen::Vector3f world_coords;
            Eigen::Vector2f displaced = Eigen::Vector2f(pixel.x, pixel.y) + normal;
            Eigen::Vector2i disp_pixel(ceil(displaced[0]), ceil(displaced[1]));
            if (pixelToWorldCoords(disp_pixel, world_coords, is_background))
                segment.points.push_back(world_coords);
        }

        //        std::vector<Eigen::Vector3f> points;
        //        simplifyDouglasPeucker(segment.points, points, 0.5);
        //        segment.points = points;

        silhouette_segments.push_back(segment);
    }

    saveBuffer("test2.png", GL_RGB);
    pixelbuffer_->doneCurrent();
}


void SilhouetteExtractor::compute_fast(std::vector<Terrain::SilhouetteSegment> &silhouette_segments)
{
    silhouette_segments.clear();
    bool use_depth_map = false;

    setupPixelBuffer();
    pixelbuffer_->makeCurrent();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    GLfloat lightpos[] = {.5, 1., 1., 0.};
    glLightfv(GL_LIGHT0, GL_POSITION, lightpos);

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!use_depth_map)
    {
        glDisable(GL_LIGHTING);
        glColorMask(false, false, false, false);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0f/2 , 1.f);
        glColor3f(0.0f, 0.0f, 1.0f);
        drawTerrain();
        glDepthMask(GL_FALSE);
        glColorMask(true, true, true, true);
        glDisable(GL_POLYGON_OFFSET_FILL);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT);
        glColor3f(0.0f, 0.0f, 0.0f);
        drawTerrainWireFrame();           // for a complete hidden-line drawing
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDepthMask(GL_TRUE);
        glDisable(GL_CULL_FACE);
        glEnable(GL_LIGHTING);
    } else
    {
        //glEnable(GL_POLYGON_OFFSET_FILL);
        //glPolygonOffset(0.0f , -2.f);
        glColor3f(1, 0, 0);
        drawTerrain();
        glDisable(GL_POLYGON_OFFSET_FILL);
    }

    cv::Mat img(height(), width(), CV_32FC1, cv::Scalar(0));
    if (use_depth_map) glReadPixels(0, 0, width(), height(), GL_DEPTH_COMPONENT, GL_FLOAT, img.data);
    else glReadPixels(0, 0, width(), height(), GL_RED, GL_FLOAT, img.data);
    if (use_depth_map)
    {
        double minval, maxval;
        cv::minMaxIdx(img, &minval, &maxval);
        if (maxval-minval > FLT_EPSILON)
            img = (img - minval)/(maxval-minval);
    }
    img.convertTo(img,CV_8UC1, 255);
    cv::flip(img, img, 0);  cv::imwrite("test1.png", img);

    if (use_depth_map)
    {
        double thres = 12;
        //cvtColor(img, img, CV_RGB2HSV);
        cv::imwrite("test1.png", img);

        cv::Canny(img, img, thres, thres*3, 3);
        cv::imwrite("test11.png", img);
    } else
        img = 255 - img;

    computeFromContour(silhouette_segments, &img);

    pixelbuffer_->doneCurrent();
    cleanupPixelBuffer();
}
*/

//void SilhouetteExtractor::compute_naive(std::vector<Terrain::SilhouetteSegment> &silhouette_segments)
//{
//    int terrain_width = terrain_->width();
//    int terrain_height = terrain_->height();
//    computeVisibleFrontFacingStatus();

//    int i = 0;
//    //#pragma omp parallel for private(i)
//    for (int j = 0; j < terrain_height-1; ++j)
//        for (i = 0; i < terrain_width-1; ++i)
//        {
//            if (!isFrontFace(i, j)) continue;

//            if (isBackFace(i+1, j))
//            {
//                Terrain::SilhouetteSegment segment;
//                segment.points.push_back(Eigen::Vector3f(i+1, j, terrain_->getAltitude(i+1, j)));
//                segment.points.push_back(Eigen::Vector3f(i+1, j+1, terrain_->getAltitude(i+1, j+1)));
//                silhouette_segments.push_back(segment);
//            }

//            if (isBackFace(i, j+1))
//            {
//                Terrain::SilhouetteSegment segment;
//                segment.points.push_back(Eigen::Vector3f(i, j+1, terrain_->getAltitude(i, j+1)));
//                segment.points.push_back(Eigen::Vector3f(i+1, j+1, terrain_->getAltitude(i+1, j+1)));
//                silhouette_segments.push_back(segment);
//            }
//        }

//    delete [] front_facing_;
//    front_facing_ = NULL;
//}

void SilhouetteExtractor::compute_naive(std::vector<Terrain::SilhouetteSegment> &silhouette_segments)
{
    int terrain_width = terrain_->width();
    int terrain_height = terrain_->height();
    computeTriangleFrontFacingStatus();

    int i = 0;
    //#pragma omp parallel for private(i)
    for (int j = 0; j < terrain_height-1; ++j)
        for (i = 0; i < terrain_width-1; ++i)
        {
            //Look at the 3 possible edges

            if (checkSilhouetteEdge(i, j, i+1, j))
            {
                Terrain::SilhouetteSegment segment;
                segment.isSilhouette = true;
                segment.points.push_back(Eigen::Vector3f(i, j, terrain_->getAltitude(i, j)));
                segment.points.push_back(Eigen::Vector3f(i+1, j, terrain_->getAltitude(i+1, j)));
                silhouette_segments.push_back(segment);
            }

            if (checkSilhouetteEdge(i, j, i, j+1))
            {
                Terrain::SilhouetteSegment segment;
                segment.isSilhouette = true;
                segment.points.push_back(Eigen::Vector3f(i, j, terrain_->getAltitude(i, j)));
                segment.points.push_back(Eigen::Vector3f(i, j+1, terrain_->getAltitude(i, j+1)));
                silhouette_segments.push_back(segment);
            }

            if (checkSilhouetteEdge(i, j, i+1, j+1))
            {
                Terrain::SilhouetteSegment segment;
                segment.isSilhouette = true;
                segment.points.push_back(Eigen::Vector3f(i, j, terrain_->getAltitude(i, j)));
                segment.points.push_back(Eigen::Vector3f(i+1, j+1, terrain_->getAltitude(i+1, j+1)));
                silhouette_segments.push_back(segment);
            }
        }

    delete [] front_facing_per_tri_;
    front_facing_per_tri_ = NULL;
}

void SilhouetteExtractor::computeTriangleFrontFacingStatus()
{
    int imax = terrain_->width()-1;
    int jmax = terrain_->height()-1;

    front_facing_per_tri_ = new FacingMode [imax*jmax*2];

    int i = 0;
#pragma omp parallel for private(i)
    for (int j = 0; j < jmax; ++j)
        for (i = 0; i < imax; ++i)
        {
            int pos = j*imax + i;
            front_facing_per_tri_[2*pos] = checkTriangleFrontFacing(i, j, i, j+1, i+1, j+1);
            front_facing_per_tri_[2*pos+1] = checkTriangleFrontFacing(i, j, i+1, j+1, i+1, j);
        }
}

SilhouetteExtractor::FacingMode SilhouetteExtractor::checkTriangleFrontFacing(int i1, int j1, int i2, int j2,
                                                                              int i3, int j3)
{
    Eigen::Vector3f point1(i1, j1, terrain_->getAltitude(i1, j1));
    Eigen::Vector3f point2(i2, j2, terrain_->getAltitude(i2, j2));
    Eigen::Vector3f point3(i3, j3, terrain_->getAltitude(i3, j3));

    Eigen::Vector3f mid = (point1+point2+point3)/3.0;
    Eigen::Vector3f projector = (mid - camera_info_.position).normalized();
    float theta = acos(camera_info_.direction.normalized().dot(projector));
    if (theta > (camera_info_.fov_in_rads/2)*1.2)
        return kInvisible;

    Eigen::Vector3f normal = (point2-point1).cross(point3-point2).normalized();
    if (normal.dot(projector) <= -FLT_EPSILON)
    {
        return kFrontFacing;
    }

    return kBackFacing;
}

bool SilhouetteExtractor::checkSilhouetteEdge(int i1, int j1, int i2, int j2)
{
    int imax = terrain_->width()-1;
    int jmax = terrain_->height()-1;

    int tri1;
    int tri2;

    Eigen::Vector3f point1(i1, j1, terrain_->getAltitude(i1, j1));
    Eigen::Vector3f point2(i2, j2, terrain_->getAltitude(i2, j2));

    Eigen::Vector3f mid = (point1+point2)/2.0;

    if (i1 == i2)
    {
        if (i1 == 0) return false;
        tri1 = 2*(j1*imax + i1-1) + 1;
        tri2 = 2*(j1*imax + i1);
    } else if (j1 == j2)
    {
        if (j1 == 0) return false;
        tri1 = 2*((j1-1)*imax + i1);
        tri2 = 2*(j1*imax + i1) + 1;
    } else if (i1 == (i2-1) && j1 == (j2-1))
    {
        if (j1 == 0) return false;
        tri1 = 2*(j1*imax + i1);
        tri2 = 2*(j1*imax + i1) + 1;
    } else
        return false;

    if (front_facing_per_tri_[tri1] == kInvisible || front_facing_per_tri_[tri2] == kInvisible)
        return false;
    if (front_facing_per_tri_[tri1] == front_facing_per_tri_[tri2])
        return false;

    if (checkVisibility(mid))
        return true;
    return false;
}

void SilhouetteExtractor::computeVisibleFrontFacingStatus()
{
    int terrain_width = terrain_->width();
    int terrain_height = terrain_->height();

    delete front_facing_;
    front_facing_ = new FacingMode [(terrain_width-1)*(terrain_height-1)];

    bool use_intersections = true ;

    if (!use_intersections)
    {
        setupPixelBuffer();
        pixelbuffer_->makeCurrent();
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHT0);
        glEnable(GL_LIGHTING);
        GLfloat lightpos[] = {.5, 1., 1., 0.};
        glLightfv(GL_LIGHT0, GL_POSITION, lightpos);

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        drawTerrain();
        updateMatrices();

        saveBuffer("test_dpth.png");
        pixelbuffer_->makeCurrent();
    }

    double begin = omp_get_wtime();

    int i = 0;
#pragma omp parallel for private(i)
    for (int j = 0; j < terrain_->height()-1; ++j)
        for (i = 0; i < terrain_->width()-1; ++i)
        {
            front_facing_[j*(terrain_width-1)+i] = kInvisible;

            Eigen::Vector3f center = getFaceCentroid(i, j);
            Eigen::Vector3f projector = center - camera_info_.position;
            //projector = camera_info_.direction;

            float theta = acos(camera_info_.direction.normalized().dot(projector.normalized()));
            if (theta > camera_info_.fov_in_rads/2)
                continue;

            front_facing_[j*(terrain_width-1)+i] = kBackFacing;

            if (terrain_->getGridNormal(i, j).dot(projector) <= -FLT_EPSILON)
            {
                if (use_intersections)
                {
                    if (checkVisibility(center))
                        front_facing_[j*(terrain_width-1)+i] = kFrontFacing;
                } else
                {
                    Eigen::Vector3d window_coords;
                    gluProject(center[0], center[1], center[2],
                               modelview_matrix_, projection_matrix_, viewport_,
                               &window_coords[0], &window_coords[1], &window_coords[2]);

                    if (window_coords[0] < 0 || window_coords[1] < 0 || window_coords[0] >= width() || window_coords[1] >= height())
                        continue;
                    float depth = 0.0;
                    glReadPixels(window_coords[0], window_coords[1], 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);

                    if (std::abs(depth-window_coords[2]) < 1e-3)
                        front_facing_[j*(terrain_width-1)+i] = kFrontFacing;
                }
            }
        }

    double end = omp_get_wtime();
    double elapsed_secs = double(end - begin);
    fprintf(stdout, "Elapsed time for checking front/back facing: %.2f secs\n", elapsed_secs);
    fprintf(stdout, "Num of threads: %d threads\n", omp_get_thread_num());
    fflush(stdout);

    if (pixelbuffer_)
    {
        pixelbuffer_->doneCurrent();
        cleanupPixelBuffer();
    }
}

bool SilhouetteExtractor::checkVisibility(Eigen::Vector3f point)
{
    float delt = 0.5f;

    std::vector<Eigen::Vector3f> intersects;
    float max_dist = (point - camera_info_.position).norm() + delt*2;
    Eigen::Vector3f ray_dir = (point - camera_info_.position).normalized();

    bool success = rayTerrainIntersect(terrain_, camera_info_.position,
                                       ray_dir, max_dist, intersects, delt);

    if (!success)
        return true;

    Eigen::Vector2f a(intersects.front()[0], intersects.front()[1]);
    Eigen::Vector2f b(point[0], point[1]);
    return ((intersects.front() - point).norm() < delt*3);
}

bool SilhouetteExtractor::isFrontFace(int i, int j)
{
    return (front_facing_[j*(terrain_->width()-1)+i] == kFrontFacing);
}

bool SilhouetteExtractor::isBackFace(int i, int j)
{
    return (front_facing_[j*(terrain_->width()-1)+i] == kBackFacing);
}

void SilhouetteExtractor::compute(const CameraInfo &camera_info,
                                  std::vector<Terrain::SilhouetteSegment> &silhouette_segments)
{
    silhouette_segments.clear();
    camera_info_ = camera_info;
    compute_naive(silhouette_segments);

    reconnectSilhouetteSegments(camera_info, silhouette_segments);
}

void SilhouetteExtractor::drawTerrainWireFrame(float precision)
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_POINTS); drawTerrain(); return;

    //    glBegin(GL_QUADS);
    //    for (int j = 0; j < terrain_->height()-1; ++j)
    //        for (int i = 0; i < terrain_->width()-1; ++i)
    //        {
    //            int nsteps  = 1.0/precision;
    //            glNormal3fv(terrain_->getGridNormal(i, j).data());

    //            for (int n = 0; n < nsteps; ++n)
    //                for (int m = 0; m < nsteps; ++m)
    //                {
    //                    float minx = i+precision*m;
    //                    float maxx = i+precision*(m+1);
    //                    float miny = j+precision*n;
    //                    float maxy = j+precision*(n+1);
    //                    glVertex3fv(Eigen::Vector3f(minx, miny, terrain_->getAltitude(minx, miny)).data());
    //                    glVertex3fv(Eigen::Vector3f(maxx, miny, terrain_->getAltitude(maxx, miny)).data());
    //                    glVertex3fv(Eigen::Vector3f(maxx, maxy, terrain_->getAltitude(maxx, maxy)).data());
    //                    glVertex3fv(Eigen::Vector3f(minx, maxy, terrain_->getAltitude(minx, maxy)).data());
    //                }
    //        }
    //    glEnd();
}

void SilhouetteExtractor::reconnectSilhouetteSegments(CameraInfo camera_info,
                                                      std::vector<Terrain::SilhouetteSegment>  &silhouette_segments)
{
    Dtts::Tree *tree = new Dtts::Tree ();

    std::set<node_t> cands;
    for (unsigned int i = 0; i < silhouette_segments.size(); ++i)
    {
        for (int j = 0; j < ((int)silhouette_segments[i].points.size())-1; ++j)
        {
            Eigen::Vector3f point1 = silhouette_segments[i].points[j];
            Eigen::Vector3f point2 = silhouette_segments[i].points[j+1];
            node_t node1(round(point1[0]), round(point1[1]));
            node_t node2(round(point2[0]), round(point2[1]));

            cands.insert(node1);
            cands.insert(node2);
        }
    }
    tree->extractFromSet(*terrain_, cands);

    //computeTreeFromSilhouetteSegments(silhouette_segments, tree);
    //tree->Smoothing();

    std::vector<node_list> branches;
    Dtts::computeBranches(*tree, branches);

    silhouette_segments.clear();

    for (unsigned int i = 0; i < branches.size(); ++i)
    {
        const node_list& branch = branches[i];
        if (branch.size() < 3)  continue;

        Terrain::SilhouetteSegment seg;
        seg.isSilhouette = true;
        for (node_list::const_iterator it3 = branch.begin(); it3!=branch.end(); it3++)
        {
           node_t b = *it3;
           Eigen::Vector3f point(b.x, b.y, terrain_->getAltitude(b.x, b.y));

           Eigen::Vector3f projector = (point - camera_info.position).normalized();
           float theta = acos(camera_info.direction.normalized().dot(projector));
           if (theta > camera_info.fov_in_rads/1.5)
           {
               if (seg.points.size() >=3)  silhouette_segments.push_back(seg);
               seg.points.clear();
           } else
                seg.points.push_back(point);
        }
        if (seg.points.size() >=3)  silhouette_segments.push_back(seg);
    }
}


void computeTreeFromSilhouetteSegments(std::vector<Terrain::SilhouetteSegment>  &silhouette_segments,
                                       Dtts::Tree *tree)
{
    tree->clear();

    for (unsigned int i = 0; i < silhouette_segments.size(); ++i)
    {
        for (int j = 0; j < ((int)silhouette_segments[i].points.size())-1; ++j)
        {
            Eigen::Vector3f point1 = silhouette_segments[i].points[j];
            Eigen::Vector3f point2 = silhouette_segments[i].points[j+1];
            node_t node1(round(point1[0]), round(point1[1]));
            node_t node2(round(point2[0]), round(point2[1]));

            tree->add_new_edge(node1, node2);
        }
    }
}

