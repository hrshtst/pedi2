#ifndef __PD_HRZ_H__
#define __PD_HRZ_H__

#include <zm/zm.h>
#include <pedi2/pd_vrt.h>

__BEGIN_DECLS

typedef struct{
  double vd;           /* desired velocity */
  double dist;         /* canonical distance of feet */

  double q1, q2;       /* system poles */
  double xzmin, xzmax; /* border of supporting region */

  double kappa;        /* curvature of referencial path */
  double rho;          /* degree of activation of nonlinear damping */
  double kr;           /* degree of initial energy exertion */

  pdVrt *vrt;
} pdHrz;

void pdHrzInit(pdHrz *hrz, pdVrt *vrt);

double pdHrzK1(pdHrz *hrz);
double pdHrzK2(pdHrz *hrz);

__END_DECLS

#endif /* __PD_HRZ_H__ */
