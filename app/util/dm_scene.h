#ifndef __DM_SCENE_H__
#define __DM_SCENE_H__

#include <roki/glrk_glx.h>
#include <roki/glrk_camera.h>
#include <roki/glrk_optic.h>
#include <roki/glrk_shape.h>
#include "dm_robot.h"

typedef struct{
  glrkCamera cam;
  glrkLight light;
  Window canvas;
  zxRegion reg;
} dmScene;

void dmSceneInit(dmScene *scene, zxWindow *parent);
void dmSceneExit(dmScene *scene);

void dmSceneLookAt(dmScene *scene, double cx, double cy, double cz, double fx, double fy, double fz);

void dmSceneResize(dmScene *scene, int x, int y, int w, int h);
void dmSceneDraw(dmScene *scene, zVec3D *force);

#endif /* __DM_SCENE_H__ */
