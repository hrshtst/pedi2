#ifndef __PD_CTRL_H__
#define __PD_CTRL_H__

#include <pedi2/pd_vrt.h>
#include <pedi2/pd_hrz.h>

__BEGIN_DECLS

typedef struct{
  pdVrt vrt;
  pdHrz tan;
  pdHrz rad;
} pdCtrl;

#define pdCtrlInit(c) do{\
  pdVrtInit( &(c)->vrt );\
  pdHrzSetup( &(c)->tan, &(c)->vrt, PD_HRZ_TAN );\
  pdHrzSetup( &(c)->rad, &(c)->vrt, PD_HRZ_RAD );\
} while(0)
#define pdCtrlDestroy(c) do{\
  pdHrzDestroy( &tan );\
  pdHrzDestroy( &rad );\
  pdHrzDestroy( &vrt );\
} while(0)
#define pdCtrlUpdate(c,z) do{\
  pdVrtUpdate( &(c)->vrt, z );\
} while(0)
#define pdCtrlZeta(c) (c)->vrt.zeta
#define pdCtrlSetPrm(c,qu1,qu2,qw1,qw2,kap,r,k) do{\
  ((pdHrzPrmTan *)(c)->tan.prm)->q1 = (qu1);\
  ((pdHrzPrmTan *)(c)->tan.prm)->q2 = (qu2);\
  ((pdHrzPrmTan *)(c)->tan.prm)->kappa = (kap);\
  ((pdHrzPrmRad *)(c)->rad.prm)->q1 = (qw1);\
  ((pdHrzPrmRad *)(c)->rad.prm)->q2 = (qw2);\
  ((pdHrzPrmRad *)(c)->rad.prm)->kappa = (kap);\
  ((pdHrzPrmRad *)(c)->rad.prm)->rho = (r);\
  ((pdHrzPrmRad *)(c)->rad.prm)->kr = (k);\
} while(0)

__END_DECLS

#endif /* __PD_CTRL_H__ */
