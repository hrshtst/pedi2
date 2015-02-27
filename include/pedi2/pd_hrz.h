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
#if 0
typedef struct{
  double xd;           /* desired position */
  double vd;           /* desired velocity */
  double q1, q2;       /* system poles */
  double xzmin, xzmax; /* border of supporting region */
  double kappa;        /* curvature of referencial path */
  double rho;          /* degree of activation of nonlinear damping */
  double kr;           /* degree of initial energy exertion */
  double dist;         /* canonical distance of feet */
} pdHrzPrm;
#endif

typedef struct{
} pdHrzCom;

typedef struct{
  byte dir;      /* direction */
  void *prm;
  pdHrzCom *com;
  pdVrt *vrt;
} pdHrz;

#define pdHrzInit(h) do{\
  (h)->dir = PD_HRZ_INVALID;\
  (h)->prm = NULL;\
  (h)->com = NULL;\
  (h)->vrt = NULL;\
} while(0)
pdHrz *pdHrzSetup(pdHrz *hrz, pdVrt *vrt, byte dir);
void pdHrzDestroy(pdHrz *hrz);

double pdHrzK1(pdHrz *hrz);
double pdHrzK2(pdHrz *hrz);

__END_DECLS

#include <pedi2/pd_hrz_tan.h> /* tangential direction */
#include <pedi2/pd_hrz_rad.h> /* radial direction */

#endif /* __PD_HRZ_H__ */
