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
  pdHrzInit( &(c)->tan, &(c)->vrt );\
  pdHrzInit( &(c)->rad, &(c)->vrt );\
} while(0)
#define pdCtrlUpdate(c,z) do{\
  pdVrtUpdateZeta( &(c)->vrt, z );\
} while(0)
#define pdCtrlZeta(c) (c)->vrt.zeta
#define pdCtrlSetParam(c,qu1,qu2,qw1,qw2,kap,r,k) do{\
  (c)->tan.q1 = (qu1);\
  (c)->tan.q2 = (qu2);\
  (c)->rad.q1 = (qw1);\
  (c)->rad.q2 = (qw2);\
  (c)->rad.kappa = (kap);\
  (c)->rad.rho = (r);\
  (c)->rad.kr = (k);\
} while(0)

__END_DECLS

#endif /* __PD_CTRL_H__ */
