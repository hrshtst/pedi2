#include <pedi2/pd_vrt.h>

void pdVrtInit(pdVrt *vrt)
{
  vrt->zd = 0;
  vrt->z = 0;
  vrt->zeta = 0;
}

void pdVrtDestroy(pdVrt *vrt)
{
  pdVrtInit( vrt );
}

void pdVrtSetRef(pdVrt *vrt, double zd)
{
  vrt->zd = zd;
}

void pdVrtUpdate(pdVrt *vrt)
{
  vrt->z = vrt->zd;
  vrt->zeta = sqrt( RK_G / vrt->zd );
}
