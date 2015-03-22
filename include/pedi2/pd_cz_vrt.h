#ifndef __PD_CZ_VRT_H__
#define __PD_CZ_VRT_H__

#include <roki/rk_g.h>
#include <zm/zm.h>

__BEGIN_DECLS

typedef struct{
  double zd;
  double z;
  double zeta;
} pdCZVrt;

void pdCZVrtInit(pdCZVrt *vrt);
void pdCZVrtDestroy(pdCZVrt *vrt);

void pdCZVrtSetRef(pdCZVrt *vrt, double zd);
void pdCZVrtUpdate(pdCZVrt *vrt);

__END_DECLS

#endif /* __PD_CZ_VRT_H__ */
