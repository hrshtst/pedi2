#include <pedi2/pd_cz_hrz.h>

void pdCZHrzInit(pdCZHrz *hrz, pdCZVrt *vrt)
{
  pdCZHrzSetRefPosX( hrz, 0 );
  pdCZHrzSetRefPosY( hrz, 0 );
  pdCZHrzSetRefTheta( hrz, 0 );
  pdCZHrzSetPosX( hrz, 0 );
  pdCZHrzSetPosY( hrz, 0 );
  pdCZHrzSetVelX( hrz, 0 );
  pdCZHrzSetVelY( hrz, 0 );
  pdCZHrzSetTheta( hrz, 0 );
  pdCZHrzSetSR( hrz, NULL );
  pdCZHrzMovInit( &hrz->_hrzm, vrt );
  hrz->refxz = 0;
  hrz->refyz = 0;
  hrz->refddx = 0;
  hrz->refddy = 0;
}

void pdCZHrzDestroy(pdCZHrz *hrz)
{
  pdCZHrzMovDestroy( &hrz->_hrzm );
  pdCZHrzSetRefPosX( hrz, 0 );
  pdCZHrzSetRefPosY( hrz, 0 );
  pdCZHrzSetRefTheta( hrz, 0 );
  pdCZHrzSetPosX( hrz, 0 );
  pdCZHrzSetPosY( hrz, 0 );
  pdCZHrzSetVelX( hrz, 0 );
  pdCZHrzSetVelY( hrz, 0 );
  pdCZHrzSetTheta( hrz, 0 );
  pdCZHrzSetSR( hrz, NULL );
}
