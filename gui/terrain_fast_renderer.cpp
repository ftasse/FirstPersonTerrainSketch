#include "terrain_fast_renderer.h"

#include <stdio.h>
#include <float.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <QImage>

#include "terrain_viewer.h"

#include "tools/shader_utils.h"

#define kNumQuadsPerSide 64
#define BUFFER_OFFSET(bytes) ((GLubyte*) NULL + (bytes))

TerrainFastRenderer::TerrainFastRenderer():
  terrain_(NULL), heightmap_texid_(0), vbo_(0), vao_(0),
  shader_program_(0), accurate_normals_(false)
{
}

Terrain*  TerrainFastRenderer::getTerrain() const
{
  return terrain_;
}

void TerrainFastRenderer::setTerrain(Terrain *terrain)
{
  terrain_ = terrain;
}

void TerrainFastRenderer::display()
{
  glPushMatrix();
  glScalef(width(), height(), 1.0f);

  Eigen::Matrix4f mvp = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f modelview_matrix, projection_matrix;
  Eigen::Matrix2i view_matrix;

  glActiveTexture( GL_TEXTURE0 + 0 );
  glBindTexture(GL_TEXTURE_2D, heightmap_texid_);

  glGetFloatv(GL_PROJECTION_MATRIX, projection_matrix.data());
  glGetFloatv(GL_MODELVIEW_MATRIX, modelview_matrix.data());
  glGetIntegerv(GL_VIEWPORT, view_matrix.data());
  mvp = projection_matrix*modelview_matrix;

  glUseProgram(shader_program_);

  glUniform1i(heightmap_location_, 0);

  // Bind detail textures
  glUniform1i(glGetUniformLocation(shader_program_, "grass"), 1);
  glUniform1i(glGetUniformLocation(shader_program_, "ground"), 2);
  glUniform1i(glGetUniformLocation(shader_program_, "perlin"), 3);
  glUniform1i(glGetUniformLocation(shader_program_, "rock"), 4);
  glUniform1i(glGetUniformLocation(shader_program_, "snow"), 5);
  glUniform1i(glGetUniformLocation(shader_program_, "grass2"), 6);

  glUniform1f(glGetUniformLocation(shader_program_, "maxHeight"), terrain_->max_altitude());


  glUniformMatrix4fv(mvp_location_, 1, GL_FALSE, mvp.data());
  glUniform2f(screensize_location_, view_matrix(0, 1), view_matrix(1, 1));
  glUniform1f(lod_factor_location_, 1.0);

  glBindVertexArray(vao_);
  if (gl4xSupported_)
    {
      glPatchParameteri(GL_PATCH_VERTICES, 4);       // tell OpenGL that every patch has 4 verts
      glDrawArrays(GL_PATCHES, 0, num_xquads*num_yquads*4); // draw a bunch of patches
    } else
    {
      glDrawArrays(GL_QUADS, 0, num_xquads*num_yquads*4);
    }
  glBindVertexArray(0);

  glUseProgram(0);

  glBindTexture(GL_TEXTURE_2D, 0);

  //        glActiveTexture(GL_TEXTURE0 + 1);
  //        glBindTexture(GL_TEXTURE_2D, details_texids[0]);
  //        glActiveTexture(GL_TEXTURE0);

  glPopMatrix();
}

void TerrainFastRenderer::init()
{
  glEnable(GL_TEXTURE_2D);

  shaders_.clear();
  shader_program_  = 0;

  gl4xSupported_ = atof(reinterpret_cast<const char*>(glGetString(GL_VERSION))) >= 4.0;

  glGenTextures(1, &heightmap_texid_);
  glGenBuffers(1, &vbo_);
  glGenVertexArrays(1, &vao_);

  if (gl4xSupported_) {
      shaders_.push_back(createShader(GL_VERTEX_SHADER,
                                      "shaders/terrain/vs_for_tes.glsl"));

      shaders_.push_back(createShader(GL_TESS_CONTROL_SHADER,
                                      "shaders/terrain/tcs.glsl"));
      shaders_.push_back(createShader(GL_TESS_EVALUATION_SHADER,
                                      "shaders/terrain/tes.glsl"));
    } else
    {
      shaders_.push_back(createShader(GL_VERTEX_SHADER,
                                      "shaders/terrain/vs.glsl"));
      std::cout << "Opengl 4 is not supported!\n" << std::flush;
    }

  shaders_.push_back(createShader(GL_FRAGMENT_SHADER,
                                  "shaders/terrain/fs.glsl"));
  shader_program_ = createProgram(shaders_);

  position_location_ = glGetAttribLocation(shader_program_, "position");
  heightmap_location_ = glGetUniformLocation(shader_program_, "terrain");
  mvp_location_ = glGetUniformLocation(shader_program_, "mvp");
  screensize_location_ = glGetUniformLocation(shader_program_, "screen_size");
  lod_factor_location_ = glGetUniformLocation(shader_program_, "lod_factor");

  if (gl4xSupported_)
  {
      num_xquads = kNumQuadsPerSide;
      num_yquads = kNumQuadsPerSide;
      setupTerrainVertexPositions(num_xquads, num_yquads);
  }

  // Init textures
  initDetailTextures();
}

void TerrainFastRenderer::cleanup()
{
  if (shader_program_ > 0) deleteProgram(shader_program_, shaders_);
  glDeleteTextures(1, &heightmap_texid_);
  glDeleteBuffers(1, &vbo_);
}

void TerrainFastRenderer::updateTerrain()
{
  if (!gl4xSupported_ && terrain_ && (num_xquads != terrain_->width() || num_yquads != terrain_->height()))
  {
      num_xquads = terrain_->width();
      num_yquads = terrain_->height();
      setupTerrainVertexPositions(num_xquads, num_yquads);
  }

  if (shader_program_ > 0)
    updateHeightmapTexture();
}

void TerrainFastRenderer::setupTerrainVertexPositions(const int quad_with, const int quad_height)
{
  int total_num_quads = (quad_with)*(num_yquads);
  std::vector<float> vertices(3*4*total_num_quads);
  int pos = 0;

  for (int y = 0; y < num_yquads; ++y)
  {
    #pragma omp for
    for (int x = 0; x < num_xquads; ++x)
      {
        vertices[pos++] = x*1.0/quad_with;
        vertices[pos++] = y*1.0/quad_height;
        vertices[pos++] = 0.0;

        vertices[pos++] = x*1.0/quad_with;
        vertices[pos++] = (y+1)*1.0/quad_height;
        vertices[pos++] = 0.0;

        vertices[pos++] = (x+1)*1.0/quad_with;
        vertices[pos++] = (y+1)*1.0/quad_height;
        vertices[pos++] = 0.0;

        vertices[pos++] = (x+1)*1.0/quad_with;
        vertices[pos++] = y*1.0/quad_height;
        vertices[pos++] = 0.0;
      }
  }

  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(float), &vertices[0], GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glBindVertexArray(vao_);
  glEnableVertexAttribArray(position_location_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glVertexAttribPointer(position_location_, 3, GL_FLOAT, false, 0, 0);
  glBindVertexArray(0);
}

void TerrainFastRenderer::updateHeightmapTexture()
{
  GLuint elevation_texid;
  glGenTextures(1, &elevation_texid);

  int nchannels = 1;
  float *data = new float [width() * height() * nchannels];

  //Set elevation
  for (int y = 0; y < terrain_->height(); ++y)
  {
    #pragma omp for
    for (int x = 0; x < terrain_->width(); ++x)
      {
        int p = (y)*terrain_->width() + x; //
        data[nchannels*p] = terrain_->getAltitude(x, terrain_->height()-1-y); //to handle flipped rendered texture
        //data[nchannels*p] = terrain_->getAltitude(x, y);
        //data[nchannels*p] -= terrain_->min_altitude();
        //data[nchannels*p] /= (terrain_->max_altitude() - terrain_->min_altitude());
      }
  }


  glBindTexture(GL_TEXTURE_2D, elevation_texid);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); //GL_REPEAT
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  //    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  //    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, width(), height(), 0, GL_RED, GL_FLOAT, data);
  glBindTexture(GL_TEXTURE_2D, 0);
  delete [] data;

  computeNormals(elevation_texid);
  glDeleteTextures(1, &elevation_texid);

  glBindTexture(GL_TEXTURE_2D, heightmap_texid_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glBindTexture(GL_TEXTURE_2D, 0);
}

//Render to height and normals info to a texture
void TerrainFastRenderer::computeNormals(unsigned int elevation_texid)
{
  glPushAttrib(GL_VIEWPORT_BIT | GL_ENABLE_BIT | GL_TRANSFORM_BIT);

  glBindTexture(GL_TEXTURE_2D, heightmap_texid_);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  //    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  //    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width(), height(), 0, GL_RGBA, GL_FLOAT, 0);

  GLuint fbo;
  glGenFramebuffers(1, &fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, heightmap_texid_, 0);
  GLenum draw_buffers_[1] = {GL_COLOR_ATTACHMENT0}; glDrawBuffers(1, draw_buffers_);

  if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    return;
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glBindTexture(GL_TEXTURE_2D, 0);

  glBindFramebuffer(GL_FRAMEBUFFER, fbo);
  glViewport(0, 0, terrain_->width(), terrain_->height());
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0.0, width(), height(), 0.0, 0.0, 1.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glScalef(width(), height(), 1.0);
  //glClearColor(1.0, 1.0, 1.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT);
  glCullFace(GL_BACK);

  std::vector<GLuint> normals_shaders;
  normals_shaders.push_back(createShader(GL_VERTEX_SHADER,
                                         "shaders/terrain/normals_vs.glsl"));
  normals_shaders.push_back(createShader(GL_FRAGMENT_SHADER,
                                         "shaders/terrain/normals_fs.glsl"));
  GLuint normals_program = createProgram(normals_shaders);
  glUseProgram(normals_program);

  glActiveTexture( GL_TEXTURE0 + 0 );
  glBindTexture(GL_TEXTURE_2D, elevation_texid);
  int terrain_loc = glGetUniformLocation(normals_program, "terrain");
  glUniform1i(terrain_loc, 0);

  Eigen::Matrix4f mvp = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f modelview_matrix, projection_matrix;
  glGetFloatv(GL_PROJECTION_MATRIX, projection_matrix.data());
  glGetFloatv(GL_MODELVIEW_MATRIX, modelview_matrix.data());
  mvp = projection_matrix*modelview_matrix;
  int mvp_loc = glGetUniformLocation(normals_program, "mvp");
  glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, mvp.data());

  //TODO: use vertex buffer for faster rendering
  glColor3f(1.0, 1.0, 0.0);
  glBegin(GL_TRIANGLES);

  for (int y = 0; y < terrain_->height(); ++y)
  {
    for (int x = 0; x < terrain_->width(); ++x)
      {
        float minx = x*1.0/width();
        float maxx = (x+1)*1.0/width();
        float miny = y*1.0/height();
        float maxy = (y+1)*1.0/height();

        glVertex3f(minx, miny, 0.0f);
        glVertex3f(maxx, miny, 0.0f);
        glVertex3f(minx, maxy, 0.0f);

        glVertex3f(maxx, miny, 0.0f);
        glVertex3f(maxx, maxy, 0.0f);
        glVertex3f(minx, maxy, 0.0f);
      }
  }
  glEnd();

  glUseProgram(0);
  deleteProgram(normals_program, normals_shaders);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glDeleteFramebuffers(1, &fbo);

  glBindTexture(GL_TEXTURE_2D, heightmap_texid_);

  float *normals_data = new float [3*width()*height()];
  glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_FLOAT, normals_data);
  terrain_->setVertexNormalsFromRGBTexture(normals_data);

  glBindTexture(GL_TEXTURE_2D, heightmap_texid_);
  int nchannels = 4;
  float *data = new float [width() * height() * nchannels];
  for (int y = 0; y < terrain_->height(); ++y)
    for (int x = 0; x < terrain_->width(); ++x)
      {
        int p = (y)*terrain_->width() + x;
        data[nchannels*p] = terrain_->getVertexNormal(x, y)[0]; //to handle flipped rendered texture
        data[nchannels*p+1] = terrain_->getVertexNormal(x, y)[1];
        data[nchannels*p+2]  = terrain_->getVertexNormal(x, y)[2];
        data[nchannels*p+3] = terrain_->getAltitude(x, y);
      }
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width(), height(), 0, GL_RGBA, GL_FLOAT, data);
  glBindTexture(GL_TEXTURE_2D, 0);
  delete [] data;


/*
  glBindTexture(GL_TEXTURE_2D, heightmap_texid_);
  float *heights = new float [4*width()*height()];
  glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA32F, GL_FLOAT, heights);
  QImage image(width(), height(), QImage::Format_RGB32);
  for (int j = 0; j < height(); ++j)
    for (int i = 0; i < width(); ++i)
    {
        int p = j*width()+i;
        float normalized_alt = (heights[p*4 + 3]-terrain_->min_altitude()) / (terrain_->max_altitude() - terrain_->min_altitude());
         // image.setPixel(i, j, qRgb(heights[p*4 + 0]*255, heights[p*4 + 1]*255, heights[p*4 + 2]*255));
         image.setPixel(i, j, qRgb(normalized_alt*255, normalized_alt*255, normalized_alt*255));
         // image.setPixel(i, height()-1-j, qRgb(heights[p]*255, heights[p]*255, heights[p]*255));
    }
  image.save("normal_map.png");
  delete heights;
  glBindTexture(GL_TEXTURE_2D, 0);
*/

  glPopAttrib();
}


void initTexture(std::string filename, int level, unsigned int id, bool mipmap = true)
{
  // Load the image
  std::cout << "Load " << filename << std::endl;
  QImage image(filename.c_str());
  QImage imageGL = QGLViewer::convertToGLFormat(image);

  std::cout << imageGL.width() << "x" << imageGL.height() << std::endl;

  // Bind the texture level (for multitexturing)
  glActiveTexture(GL_TEXTURE0 + int(level));
  glBindTexture(GL_TEXTURE_2D, id);

  // Load the texture
  glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA,
                imageGL.width(),
                imageGL.height(),
                0,
                GL_RGBA,
                GL_UNSIGNED_BYTE,
                imageGL.bits() );

  // Texture properties
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,GL_REPEAT);

  // Generate the mipmap
  if (mipmap)
    {
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
      glGenerateMipmap(GL_TEXTURE_2D);
    }

  // Unbind the texture and the level
  glActiveTexture(GL_TEXTURE0);
}


void TerrainFastRenderer::initDetailTextures()
{
  // Allocate the textures
  glGenTextures(6, details_texids);

  // For each texture, load the image and allocate the texture
  initTexture("terrain_textures/grass.jpg", 1, details_texids[0]);
  initTexture("terrain_textures/ground.jpg", 2, details_texids[1]);
  initTexture("terrain_textures/perlin.jpg", 3, details_texids[2], false);
  initTexture("terrain_textures/rock.jpg", 4, details_texids[3]);
  initTexture("terrain_textures/snow.jpg", 5, details_texids[4]);
  initTexture("terrain_textures/grass2.jpg", 6, details_texids[5]);
}


void TerrainFastRenderer::initDiffuseTexture()
{
  // Allocate the textures
  glGenTextures(1, &diffuse_texid);
}



