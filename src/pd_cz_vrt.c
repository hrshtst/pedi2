#include <pedi2/pd_cz_vrt.h>

void pdCZVrtInit(pdCZVrt *vrt)
{
  vrt->zd = 0;
  vrt->q1 = 0;
  vrt->q2 = 0;
  vrt->mass  = 0;
  vrt->fz = 0;

  vrt->acc = 0;
  vrt->zeta = 0;
}

void pdCZVrtDestroy(pdCZVrt *vrt)
{
  pdCZVrtInit( vrt );
}

void pdCZVrtUpdate(pdCZVrt *vrt)
{
  vrt->z = vrt->zd;
  vrt->zeta = sqrt( RK_G / vrt->zd );
}
