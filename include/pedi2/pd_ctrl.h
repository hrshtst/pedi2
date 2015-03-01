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
} while (0)
#define pdCtrlDestroy(c) do{\
  pdHrzDestroy( &(c)->tan );\
  pdHrzDestroy( &(c)->rad );\
  pdVrtDestroy( &(c)->vrt );\
} while (0)
#define pdCtrlUpdate(c,z) do{\
  pdVrtUpdate( &(c)->vrt );\
  pdHrzUpdate( &(c)->tan );\
  pdHrzUpdate( &(c)->rad );\
} while (0)
#define pdCtrlZeta(c) (c)->vrt.zeta
#define pdCtrlPrmTan(c) ((pdHrzPrmTan *)(c)->tan.prm)
#define pdCtrlPrmRad(c) ((pdHrzPrmRad *)(c)->rad.prm)
#define pdCtrlSetPrm(c,qu1,qu2,qw1,qw2,kap,r,k) do{\
  pdCtrlPrmTan(c)->q1 = (qu1);\
  pdCtrlPrmTan(c)->q2 = (qu2);\
  pdCtrlPrmTan(c)->kappa = (kap);\
  pdCtrlPrmRad(c)->q1 = (qw1);\
  pdCtrlPrmRad(c)->q2 = (qw2);\
  pdCtrlPrmRad(c)->kappa = (kap);\
  pdCtrlPrmRad(c)->rho = (r);\
  pdCtrlPrmRad(c)->kr = (k);\
} while(0)
#define pdCtrlSetRef(c,vud,vwd,d) do{\
  pdCtrlPrmTan(c)->vd = (vud);\
  pdCtrlPrmRad(c)->vd = (vwd);\
  pdCtrlPrmRad(c)->dist = (d);\
} while (0)
#define pdCtrlZMPTan(c) pdHrzZMP( &(c)->tan )
#define pdCtrlZMPRad(c) pdHrzZMP( &(c)->rad )

__END_DECLS

#endif /* __PD_CTRL_H__ */
