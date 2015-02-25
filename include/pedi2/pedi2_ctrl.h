#ifndef __PEDI2_CTRL_H__
#define __PEDI2_CTRL_H__

#include <roki/rk_g.h>
#include <zm/zm.h>

typedef struct{
  double z;
  double zeta;
} pedi2Vert;

void pedi2VertUpdate(pedi2Vert *vert);

typedef struct{
  /* control input */
  double uz;
  double wz;
  /* control parameter */
  double qu1;
  double qu2;
  double qw1;
  double qw2;
  double kappa;
  double rho;
  double kr;
  double vd;
  double w;

  pedi2Vert *vert;
} pedi2Ctrl;

void pedi2CtrlInit(pedi2Ctrl *c);
void pedi2CtrlZMP(pedi2Ctrl *c, double du, double vu, double dw, double vw);

#endif /* __PEDI2_CTRL_H__ */
