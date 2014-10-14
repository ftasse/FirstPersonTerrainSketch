#include "offline_terrain_renderer.h"

OfflineTerrainRenderer::OfflineTerrainRenderer(Terrain *terrain):
    OfflineRenderer(), terrain_(terrain)
{
}

OfflineTerrainRenderer::~OfflineTerrainRenderer()
{
    terrain_ = NULL;
}

void OfflineTerrainRenderer::drawTerrain()
{
    glPushMatrix();

    bool has_vert_normals = terrain_->hasVertexNormals();

    glBegin(GL_TRIANGLES);
    for (int j = 0; j < terrain_->height()-1; ++j)
        for (int i = 0; i < terrain_->width()-1; ++i)
        {
            if (has_vert_normals) glNormal3fv(terrain_->getVertexNormal(i, j).data());
            glVertex3fv(Eigen::Vector3f(i, j, terrain_->getAltitude(i, j)).data());
            if (has_vert_normals) glNormal3fv(terrain_->getVertexNormal(i+1, j).data());
            glVertex3fv(Eigen::Vector3f(i+1, j, terrain_->getAltitude(i+1, j)).data());
            if (has_vert_normals) glNormal3fv(terrain_->getVertexNormal(i, j+1).data());
            glVertex3fv(Eigen::Vector3f(i, j+1, terrain_->getAltitude(i, j+1)).data());

            if (has_vert_normals) glNormal3fv(terrain_->getVertexNormal(i+1, j).data());
            glVertex3fv(Eigen::Vector3f(i+1, j, terrain_->getAltitude(i+1, j)).data());
            if (has_vert_normals) glNormal3fv(terrain_->getVertexNormal(i+1, j+1).data());
            glVertex3fv(Eigen::Vector3f(i+1, j+1, terrain_->getAltitude(i+1, j+1)).data());
            if (has_vert_normals) glNormal3fv(terrain_->getVertexNormal(i, j+1).data());
            glVertex3fv(Eigen::Vector3f(i, j+1, terrain_->getAltitude(i, j+1)).data());
        }
    glEnd();

    glPopMatrix();
}

Eigen::Vector3f OfflineTerrainRenderer::getFaceCentroid(int i, int j)
{
    Eigen::Vector3f center = Eigen::Vector3f(i, j, terrain_->getAltitude(i, j));
    center += Eigen::Vector3f(i+1, j+1, terrain_->getAltitude(i+1, j+1));
    center += Eigen::Vector3f(i, j+1, terrain_->getAltitude(i, j+1));
    center += Eigen::Vector3f(i+1, j, terrain_->getAltitude(i+1, j));
    center /= 4.0;
    return center;
}

void OfflineTerrainRenderer::unProject(std::vector<Eigen::Vector2i> &window_points,
                                std::vector<Eigen::Vector3f> &unprojected_points)
{
    setupPixelBuffer();
    pixelbuffer_->makeCurrent();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
    drawTerrain();

//    saveBuffer("snapshot.png");

    unprojected_points.clear();
    updateMatrices();

    for (unsigned int i = 0; i < window_points.size(); ++i)
    {
        Eigen::Vector3f unprojected;
        bool is_background;
        pixelToWorldCoords(window_points[i], unprojected, is_background);
        unprojected_points.push_back(unprojected);
    }

    pixelbuffer_->doneCurrent();
    cleanupPixelBuffer();
}
