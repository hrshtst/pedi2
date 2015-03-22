#ifndef __PD_CZ_H__
#define __PD_CZ_H__

#include <pedi2/pd_vrt.h>
#include <pedi2/pd_hrz.h>

__BEGIN_DECLS

typedef struct{
  pdVrt vrt;
  pdHrz tan;
  pdHrz rad;
} pdCZ;

#define pdCZInit(c) do{\
  pdVrtInit( &(c)->vrt );\
  pdHrzSetup( &(c)->tan, &(c)->vrt, PD_HRZ_TAN );\
  pdHrzSetup( &(c)->rad, &(c)->vrt, PD_HRZ_RAD );\
} while (0)
#define pdCZDestroy(c) do{\
  pdHrzDestroy( &(c)->tan );\
  pdHrzDestroy( &(c)->rad );\
  pdVrtDestroy( &(c)->vrt );\
} while (0)
#define pdCZZeta(c) (c)->vrt.zeta
#define pdCZPrmTan(c) ((pdHrzPrmTan *)(c)->tan.prm)
#define pdCZPrmRad(c) ((pdHrzPrmRad *)(c)->rad.prm)
#define pdCZSetPrm(c,qu1,qu2,qw1,qw2,kap,r,k) do{\
  pdCZPrmTan(c)->q1 = (qu1);\
  pdCZPrmTan(c)->q2 = (qu2);\
  pdCZPrmTan(c)->kappa = (kap);\
  pdCZPrmRad(c)->q1 = (qw1);\
  pdCZPrmRad(c)->q2 = (qw2);\
  pdCZPrmRad(c)->kappa = (kap);\
  pdCZPrmRad(c)->rho = (r);\
  pdCZPrmRad(c)->kr = (k);\
} while(0)
#define pdCZSetRefVrt(c,zd) do{\
  pdVrtSetRef( &(c)->vrt, zd );\
} while (0)
#define pdCZSetRefHrz(c,vud,vwd,d) do{\
  pdCZPrmTan(c)->vd = (vud);\
  pdCZPrmRad(c)->vd = (vwd);\
  pdCZPrmRad(c)->dist = (d);\
} while (0)
#define pdCZKappa(c) pdCZPrmTan(c)->kappa
#define pdCZZMPTan(c) pdHrzZMP( &(c)->tan )
#define pdCZZMPRad(c) pdHrzZMP( &(c)->rad )
#define pdCZUpdate(c,du,vu,dw,vw) do{\
  pdVrtUpdate( &(c)->vrt );\
  pdHrzUpdate( &(c)->tan, du, vu, dw, vw );\
  pdHrzUpdate( &(c)->rad, du, vu, dw, vw );\
} while (0)

__END_DECLS

#endif /* __PD_CZ_H__ */
