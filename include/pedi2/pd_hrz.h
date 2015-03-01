#ifndef __PD_HRZ_H__
#define __PD_HRZ_H__

#include <zm/zm.h>
#include <pedi2/pd_vrt.h>

__BEGIN_DECLS

enum{
  PD_HRZ_INVALID=-1,
  PD_HRZ_TAN=0, /* tangential direction */
  PD_HRZ_RAD,   /* radial direction */
};

typedef struct{
  void (*_setprm)(void*,void*);
  double (*_k1)(void*);
  double (*_k2)(void*);
  void (*_update)(void*,double,double,double,double);
  double (*_zmp)(void*);
} pdHrzCom;

typedef struct{
  byte dir;      /* direction */
  void *prm;
  pdHrzCom *com;
} pdHrz;

#define pdHrzInit(h) do{\
  (h)->dir = PD_HRZ_INVALID;\
  (h)->prm = NULL;\
  (h)->com = NULL;\
} while(0)
pdHrz *pdHrzSetup(pdHrz *hrz, pdVrt *vrt, byte dir);
void pdHrzDestroy(pdHrz *hrz);

#define pdHrzPrm(h) (h)->prm
#define pdHrzSetPrm(h,p) (h)->com->_setprm( (h)->prm, p )

#define pdHrzK1(h) (h)->com->_k1( (h)->prm )
#define pdHrzK2(h) (h)->com->_k2( (h)->prm )
#define pdHrzUpdate(h,du,vu,dw,vw) \
  (h)->com->_update( (h)->prm, du, vu, dw, vw )
#define pdHrzZMP(h) (h)->com->_zmp( (h)->prm )

__END_DECLS

#include <pedi2/pd_hrz_tan.h> /* tangential direction */
#include <pedi2/pd_hrz_rad.h> /* radial direction */

#endif /* __PD_HRZ_H__ */
