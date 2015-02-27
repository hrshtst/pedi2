#ifndef __PD_HRZ_RAD_H__
#define __PD_HRZ_RAD_H__

__BEGIN_DECLS

typedef struct{
  double vd;           /* desired velocity */
  double q1, q2;       /* system poles */
  double xzmin, xzmax; /* border of supporting region */
  double kappa;        /* curvature of referencial path */
  double rho;          /* degree of activation of nonlinear damping */
  double kr;           /* degree of initial energy exertion */
  double dist;         /* canonical distance of feet */

  double wz;           /* desired ZMP position */
  double wd;           /* desired COM position */
} pdHrzPrmRad;

pdHrz *pdHrzSetupRad(pdHrz *hrz);

__END_DECLS

#endif /* __PD_HRZ_RAD_H__ */
