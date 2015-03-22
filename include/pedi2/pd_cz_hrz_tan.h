#ifndef __PD_CZ_HRZ_TAN_H__
#define __PD_CZ_HRZ_TAN_H__

__BEGIN_DECLS

typedef struct{
  double vd;           /* desired velocity */
  double q1, q2;       /* system poles */
  double xzmin, xzmax; /* border of supporting region */
  double kappa;        /* curvature of referencial path */

  double uz;           /* desired ZMP position */
  double ud;           /* desired COM position */

  pdCZVrt *vrt;
} pdCZHrzPrmTan;

pdCZHrz *pdCZHrzSetupTan(pdCZHrz *hrz, pdCZVrt *vrt);

__END_DECLS

#endif /* __PD_CZ_HRZ_TAN_H__ */
