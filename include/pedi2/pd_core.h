#ifndef __PD_CORE_H__
#define __PD_CORE_H__

#include <zm/zm_ode.h>
#include <pedi2/pd_cz.h>
#include <pedi2/pd_foot.h>
#include <pedi2/pd_robot.h>

__BEGIN_DECLS

typedef struct{
  double qu1, qu2;
  double qw1, qw2;
  double kappa, rho, kr;
  double zd;
  double vud, vwd, dist;
  double lfh, rfh;
} pdCommand;

void pdCommandInit(pdCommand *com);
void pdCommandDefaultInit(pdCommand *com);
void pdCommandExit(pdCommand *com);

typedef struct{
  double t;
  zVec p;
  zODE ode;
} pdODESolver;

typedef struct{
  double x[2], y[2];
  double nx[2], ny[2];
  pdCZ cz;
  pdFoot lf, rf;
  double xz, yz;
  double xd, yd;
  double theta;
  double nxd, nyd;
  double ntheta;
  double adx, ady;
  pdRobot robot;
  pdCommand *com;
  pdODESolver solver;
} pdCore;

void pdCoreInit(pdCore *core, pdCommand *com);
void pdCoreLoad(pdCore *core, char *model_file, char *conf_file);
void pdCoreExit(pdCore *core);

void pdCoreSetCommand(pdCore *core, pdCommand *com);
void pdCoreSetState(pdCore *core, double *x, double *y, double theta);
void pdCoreUpdate(pdCore *core, double dt);
#define pdCoreGetJointSize(c) pdRobotGetJointSize(&(c)->robot)
#define pdCoreGetJointDis(c,v) pdRobotGetJointDis(&(c)->robot,v)

void pdCoreFWrite(pdCore *core, FILE *fp);

__END_DECLS

#endif /* __PD_CORE_H__ */
