#ifndef __PD_CZ_H__
#define __PD_CZ_H__

#include <pedi2/pd_cz_vrt.h>
#include <pedi2/pd_cz_hrz.h>

__BEGIN_DECLS

typedef struct{
  pdCZVrt vrt;
  pdCZHrz tan;
  pdCZHrz rad;
} pdCZ;

#define pdCZInit(c) do{\
  pdCZVrtInit( &(c)->vrt );\
  pdCZHrzSetup( &(c)->tan, &(c)->vrt, PD_CZ_HRZ_TAN );\
  pdCZHrzSetup( &(c)->rad, &(c)->vrt, PD_CZ_HRZ_RAD );\
} while (0)
#define pdCZDestroy(c) do{\
  pdCZHrzDestroy( &(c)->tan );\
  pdCZHrzDestroy( &(c)->rad );\
  pdCZVrtDestroy( &(c)->vrt );\
} while (0)
#define pdCZZeta(c) (c)->vrt.zeta
#define pdCZPrmTan(c) ((pdCZHrzPrmTan *)(c)->tan.prm)
#define pdCZPrmRad(c) ((pdCZHrzPrmRad *)(c)->rad.prm)
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
  pdCZVrtSetRef( &(c)->vrt, zd );\
} while (0)
#define pdCZSetRefHrz(c,vud,vwd,d) do{\
  pdCZPrmTan(c)->vd = (vud);\
  pdCZPrmRad(c)->vd = (vwd);\
  pdCZPrmRad(c)->dist = (d);\
} while (0)
#define pdCZKappa(c) pdCZPrmTan(c)->kappa
#define pdCZZMPTan(c) \
  pdCZHrzZMP( &(c)->tan )
#define pdCZZMPRad(c) \
  pdCZHrzZMP( &(c)->rad )
#define pdCZAccTan(c) \
  pdCZHrzAcc( &(c)->tan )
#define pdCZAccRad(c) \
  pdCZHrzAcc( &(c)->rad )
#define pdCZUpdate(c,du,vu,dw,vw) do{\
  pdCZVrtUpdate( &(c)->vrt );\
  pdCZHrzUpdate( &(c)->tan, du, vu, dw, vw );\
  pdCZHrzUpdate( &(c)->rad, du, vu, dw, vw );\
} while (0)

void pdCZZMPPhase(pdCZ *c, double dw, double vw, zComplex *pz);

__END_DECLS

#endif /* __PD_CZ_H__ */
