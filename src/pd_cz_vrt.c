#include <pedi2/pd_cz_vrt.h>

void pdCZVrtInit(pdCZVrt *vrt)
{
  vrt->_zd = 0;
  vrt->_q1 = 0;
  vrt->_q2 = 0;
  vrt->mass  = 0;

  vrt->_reffz = 0;
  vrt->_refacc = 0;
  vrt->zeta = 0;
}

void pdCZVrtDestroy(pdCZVrt *vrt)
{
  pdCZVrtInit( vrt );
}

void pdCZVrtUpdate(pdCZVrt *vrt)
{
  vrt->z = vrt->_zd;
  vrt->zeta = sqrt( RK_G / vrt->_zd );
}
