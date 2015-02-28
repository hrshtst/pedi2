#ifndef __PD_VRT_H__
#define __PD_VRT_H__

#include <roki/rk_g.h>
#include <zm/zm.h>

__BEGIN_DECLS

typedef struct{
  double z;
  double zeta;
} pdVrt;

void pdVrtInit(pdVrt *vrt);
void pdVrtDestroy(pdVrt *vrt);

void pdVrtUpdate(pdVrt *vrt, double z);

__END_DECLS

#endif /* __PD_VRT_H__ */
