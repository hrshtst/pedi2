#include <pedi2/pd_hrz.h>

static pdHrz *(* pd_hrz_setup[])(pdHrz*,pdVrt*) = {
  pdHrzSetupTan,
  pdHrzSetupRad,
};

pdHrz *pdHrzSetup(pdHrz *h, pdVrt *v, byte dir)
{
  if( dir < PD_HRZ_TAN || dir > PD_HRZ_RAD ){
    ZRUNERROR( "invalid direction specified - %d", dir);
    return NULL;
  }
  pdHrzInit( h );
  if( !pd_hrz_setup[( (h)->dir = dir )]( h, v ) ){
    ZRUNERROR( "cannot setup horizontal controller" );
    pdHrzDestroy( h );
    return NULL;
  }
  return h;
}

void pdHrzDestroy(pdHrz *h)
{
  zFree( h->prm );
  pdHrzInit( h );
}

#if 0
double pdHrzK1(pdHrz *hrz)
{
  return hrz->prm->q1 * hrz->prm->q2;
}

double pdHrzK2(pdHrz *hrz)
{
  return ( hrz->prm->q1 + hrz->prm->q2 ) / hrz->vrt->zeta;
}
#endif
