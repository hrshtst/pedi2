#include <pedi2/pd_cz_vrt.h>

void pdCZVrtInit(pdCZVrt *vrt)
{
  vrt->zd = 0;
  vrt->z = 0;
  vrt->zeta = 0;
}

void pdCZVrtDestroy(pdCZVrt *vrt)
{
  pdCZVrtInit( vrt );
}

void pdCZVrtSetRef(pdCZVrt *vrt, double zd)
{
  vrt->zd = zd;
}

void pdCZVrtUpdate(pdCZVrt *vrt)
{
  vrt->z = vrt->zd;
  vrt->zeta = sqrt( RK_G / vrt->zd );
}
