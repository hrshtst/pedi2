#ifndef __PD_CZ_HRZ_H__
#define __PD_CZ_HRZ_H__

#include <zm/zm.h>
#include <pedi2/pd_cz_vrt.h>

__BEGIN_DECLS

enum{
  PD_CZ_HRZ_INVALID=-1,
  PD_CZ_HRZ_TAN=0, /* tangential direction */
  PD_CZ_HRZ_RAD,   /* radial direction */
};

typedef struct{
  void (*_setprm)(void*,void*);
  double (*_k1)(void*);
  double (*_k2)(void*);
  double (*_zmp)(void*,double,double,double,double);
  double (*_acc)(void*,double,double,double,double);
  void (*_update)(void*,double,double,double,double);
} pdCZHrzCom;

typedef struct{
  byte dir;      /* direction */
  void *prm;
  pdCZHrzCom *com;
} pdCZHrz;

#define pdCZHrzInit(h) do{\
  (h)->dir = PD_CZ_HRZ_INVALID;\
  (h)->prm = NULL;\
  (h)->com = NULL;\
} while(0)
pdCZHrz *pdCZHrzSetup(pdCZHrz *hrz, pdCZVrt *vrt, byte dir);
void pdCZHrzDestroy(pdCZHrz *hrz);

#define pdCZHrzPrm(h) (h)->prm
#define pdCZHrzSetPrm(h,p) (h)->com->_setprm( (h)->prm, p )

#define pdCZHrzK1(h) (h)->com->_k1( (h)->prm )
#define pdCZHrzK2(h) (h)->com->_k2( (h)->prm )
#define pdCZHrzZMP(h,du,vu,dw,vw) \
  (h)->com->_zmp( (h)->prm, du, vu, dw, vw )
#define pdCZHrzAcc(h,du,vu,dw,vw) \
  (h)->com->_acc( (h)->prm, du, vu, dw, vw )
#define pdCZHrzUpdate(h,du,vu,dw,vw) \
  (h)->com->_update( (h)->prm, du, vu, dw, vw )

__END_DECLS

#include <pedi2/pd_cz_hrz_tan.h> /* tangential direction */
#include <pedi2/pd_cz_hrz_rad.h> /* radial direction */

#endif /* __PD_CZ_HRZ_H__ */
