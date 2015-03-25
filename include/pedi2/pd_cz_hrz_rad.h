#ifndef __PD_CZ_HRZ_RAD_H__
#define __PD_CZ_HRZ_RAD_H__

__BEGIN_DECLS

typedef struct{
  double xd;           /* desired COM position */
  double vd;           /* desired COM velocity */
  double q1, q2;       /* system poles */
  double xzmin, xzmax; /* border of supporting region */
  double kappa;        /* curvature of referencial path */
  double rho;          /* degree of activation of nonlinear damping */
  double kr;           /* degree of initial energy exertion */
  double dist;         /* canonical distance of feet */

  double zmp;          /* ZMP position */
  double acc;          /* COM acceleration */

  pdCZVrt *vrt;
} pdCZHrzPrmRad;

pdCZHrz *pdCZHrzSetupRad(pdCZHrz *hrz, pdCZVrt *vrt);

__END_DECLS

#endif /* __PD_CZ_HRZ_RAD_H__ */
