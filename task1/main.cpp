
#include <GLFW/glfw3.h>
#include "delfem2/glfw/viewer2.h"
#include "delfem2/color.h"
#include <random>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <valarray>

// print out error
static void error_callback(int error, const char* description){
  fputs(description, stderr);
}

using fpvec = std::valarray<float>;

class CParticle{
public:
  fpvec pos = {0.f, 0.f};
  fpvec velo = {0.f, 0.f};
  float color[3] = {0.f, 0.f, 0.f};
};

void draw(
    const std::vector<CParticle>& aParticle)
{
  ::glLineWidth(2);

  // draw boundary
  ::glBegin(GL_LINE_LOOP);
  ::glColor3f(0.f, 0.f, 0.f);
  ::glVertex2f(0.f, 0.f);
  ::glVertex2f(1.f, 0.f);
  ::glVertex2f(1.f, 1.f);
  ::glVertex2f(0.f, 1.f);
  ::glEnd();

  { // draw obstacle circle (center=(0.5,0.5) radius=0.2 )
    ::glBegin(GL_LINE_LOOP);
    ::glColor3f(0.f, 0.f, 0.f);
    const unsigned int ndiv = 64;
    const float dr = 2.f*M_PI/ndiv;
    for(unsigned int idiv=0;idiv<ndiv;++idiv){
      ::glVertex2f(
          0.5+0.2*cos(dr*float(idiv)),
          0.5+0.2*sin(dr*float(idiv)) );
    }
    ::glEnd();
  }

  // draw points
  ::glPointSize(10);
  ::glBegin(GL_POINTS);
  for(const auto & ip : aParticle){
    ::glColor3fv(ip.color);
    ::glVertex2d(ip.pos[0],ip.pos[1]);
  }
  ::glEnd();
}

void simulate(
    std::vector<CParticle>& aParticle,
    float dt)
{
  for(auto & p : aParticle) { // loop for all the particles
    using std::pow;
    using std::sqrt;

    const static fpvec c{0.5f, 0.5f};
    constexpr float r{0.2f};

    // temporary calc the next position
    fpvec post_pos = p.pos + dt * p.velo;

    fpvec c2pos = p.pos - c;
    float sq_arc2pos = pow(c2pos, 2).sum() - r * r;
    // see if the particle is in the circle
    bool in_circle = sq_arc2pos < 0,
         post_in_circle = (pow(post_pos - c, 2).sum() < r * r);

    // when crossing the circle
    if (in_circle != post_in_circle) {
      float dot_c2pos_velo = (c2pos * p.velo).sum();
      float sq_speed = pow(p.velo, 2).sum();

      // when collision w/ circle
      float hit_dt = (-dot_c2pos_velo
                          + (in_circle ? 1 : -1) * sqrt(dot_c2pos_velo * dot_c2pos_velo - sq_speed * sq_arc2pos))
                      / sq_speed;

      fpvec c2_hitpos = c2pos + hit_dt * p.velo;
      p.velo -= 2 * (dot_c2pos_velo + hit_dt * sq_speed) / (r * r) * c2_hitpos;
      p.pos = c + c2_hitpos + (dt - hit_dt) * p.velo;

      continue;
    }

    // update positions
    p.pos = post_pos;
    // ------------------------
    // solve collisions below
    if( p.pos[0] < 0 ){ // left wall
      p.pos[0] = -p.pos[0];
      p.velo[0] = -p.velo[0];
    }
    if( p.pos[1] < 0 ){ // bottom wall
      p.pos[1] = -p.pos[1];
      p.velo[1] = -p.velo[1];
    }
    if( p.pos[0] > 1 ){ // left wall
      p.pos[0] = 2-p.pos[0];
      p.velo[0] = -p.velo[0];
    }
    if( p.pos[1] > 1 ){ // top wall
      p.pos[1] = 2-p.pos[1];
      p.velo[1] = -p.velo[1];
    }
  }
}

int main()
{
  const unsigned int N = 50; // number of points
  const float dt = 0.01;
  std::vector<CParticle> aParticle;
  { // initialize particle
    std::mt19937 rndeng(std::random_device{}());
    std::uniform_real_distribution<float> dist01(0,1);
    aParticle.resize(N);
    for(auto & p : aParticle){
      // set position
      p.pos[0] = dist01(rndeng);
      p.pos[1] = dist01(rndeng);
      // set color
      delfem2::GetRGB_HSV(
          p.color[0], p.color[1], p.color[2],
          dist01(rndeng), 1.f, 1.f);
      // set velocity
      p.velo[0] = dist01(rndeng)*2.f-1.f;
      p.velo[1] = dist01(rndeng)*2.f-1.f;
      const float lenvelo = sqrt(p.velo[0]*p.velo[0]+p.velo[1]*p.velo[1]);
      p.velo[0] /= lenvelo;
      p.velo[1] /= lenvelo;
    }
  }
  delfem2::glfw::CViewer2 viewer;
  { // specify view
    viewer.view_height = 1.0;
    viewer.trans[0] = -0.5;
    viewer.trans[1] = -0.5;
    viewer.title = "task1";
  }
  glfwSetErrorCallback(error_callback);
  // -----------------------------------
  // below: opengl starts from here
  if ( !glfwInit() ) { exit(EXIT_FAILURE); }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
  viewer.InitGL();

  while (!glfwWindowShouldClose(viewer.window))
  {
    viewer.DrawBegin_oldGL();
    simulate(aParticle,dt);
    draw(aParticle);
    viewer.SwapBuffers();
    glfwPollEvents();
  }
  glfwDestroyWindow(viewer.window);
  glfwTerminate();
  exit(EXIT_SUCCESS);
}
