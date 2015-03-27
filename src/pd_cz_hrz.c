#include <pedi2/pd_cz_hrz.h>

static pdCZHrz *(* pd_hrz_setup[])(pdCZHrz*,pdCZVrt*) = {
  pdCZHrzSetupTan,
  pdCZHrzSetupRad,
};

pdCZHrz *pdCZHrzSetup(pdCZHrz *h, pdCZVrt *v, byte dir)
{
  if( dir < PD_CZ_HRZ_TAN || dir > PD_CZ_HRZ_RAD ){
    ZRUNERROR( "invalid direction specified - %d", dir);
    return NULL;
  }
  pdCZHrzInit( h );
  if( !pd_hrz_setup[( (h)->dir = dir )]( h, v ) ){
    ZRUNERROR( "cannot setup horizontal controller" );
    pdCZHrzDestroy( h );
    return NULL;
  }
  return h;
}

void pdCZHrzDestroy(pdCZHrz *h)
{
  zFree( h->prm );
  pdCZHrzInit( h );
}
