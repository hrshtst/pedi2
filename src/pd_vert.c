#include <pedi2/pd_vrt.h>

void pdVrtInit(pdVrt *vrt)
{
  vrt->z = 0;
}

void pdVrtUpdateZeta(pdVrt *vrt, double z)
{
  vrt->z = z;
  vrt->zeta = sqrt( RK_G / vrt->z );
}
