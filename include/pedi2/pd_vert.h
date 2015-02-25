#ifndef __PD_VERT_H__
#define __PD_VERT_H__

#include <roki/rk_g.h>
#include <zm/zm.h>

__BEGIN_DECLS

typedef struct{
  double z;
  double zeta;
} pdVert;

void pdVertInit(pdVert *vert);
void pdVertUpdateZeta(pdVert *vert, double z);

__END_DECLS

#endif /* __PD_VERT_H__ */
