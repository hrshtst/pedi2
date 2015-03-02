#ifndef __SCENE_H__
#define __SCENE_H__

#include <zeo/zeo_vec3d.h>
#include <roki/glrk_glx.h>
#include <roki/glrk_camera.h>
#include <roki/glrk_optic.h>
#include <roki/glrk_shape.h>

__BEGIN_DECLS

typedef struct{
  glrkCamera cam;
  glrkLight light;
  Window canvas;
  zxRegion reg;
  int gauge;
} Scene;

void initScene(Scene *scene, zxWindow *parent);
void exitScene(Scene *scene);

void lookAt(Scene *scene, double cx, double cy, double cz, double fx, double fy, double fz);

void resizeScene(Scene *scene, int x, int y, int w, int h);
void drawScene(Scene *scene, zVec3D *com, zVec3D *zmp);

__END_DECLS

#endif /* __SCENE_H__ */
