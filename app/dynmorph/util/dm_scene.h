#ifndef __DM_SCENE_H__
#define __DM_SCENE_H__

#include <roki/rkgl_glx.h>
#include <roki/rkgl_camera.h>
#include <roki/rkgl_optic.h>
#include <roki/rkgl_shape.h>
#include <roki/rkgl_chain.h>

typedef struct{
  rkglCamera cam;
  rkglLight light;
  Window canvas;
  zxRegion reg;
} dmScene;

void dmSceneInit(dmScene *scene, zxWindow *parent);
void dmSceneExit(dmScene *scene);

void dmSceneLookAt(dmScene *scene, double cx, double cy, double cz, double fx, double fy, double fz);

void dmSceneResize(dmScene *scene, int x, int y, int w, int h);
void dmSceneDraw(dmScene *scene, zVec dis, zVec3D *force);

/* scene */
void dmGLInit(const char* model_file);
void dmGLExit(void);
void dmGLGauge(void);
void dmGLRobot(void);
void dmGLSupportRegion(void);

#endif /* __DM_SCENE_H__ */
