#include <pedi2/pd_foot.h>

void pdFootInit(pdFoot *f, pdCZHrz *cz, byte dir, double dt)
{
  pdFootUWInit( pdFootUWPtr( f ), pdCZHrzUWPtr( cz ) );
  pdFootZInit( pdFootZPtr( f ), pdCZHrzUWPtr( cz ) );
  pdFootSetPos( f, 0, 0, 0 );
  pdFootSetDesPos( f, 0, 0, 0 );
  pdFootSetAtt( f, 0, 0, 0 );
  pdFootSetDesAtt( f, 0, 0, 0 );
  pdFootSetSR( f, NULL );
  pdFootCZPtr( f ) = cz;
  pdFootResetTime( f );
  pdFootSetTimeStep( f, dt );
  pdFootSetTrXK( f, 0 );
  pdFootSetTrXC( f, 0 );
  pdFootSetTrXOld( f, 0 );
  pdFootSetTrYK( f, 0 );
  pdFootSetTrYC( f, 0 );
  pdFootSetTrYOld( f, 0 );
  pdFootSetTrZK( f, 0 );
  pdFootSetTrZC( f, 0 );
  pdFootSetTrZOld( f, 0 );
  pdFootRefPosX( f ) = 0;
  pdFootRefPosY( f ) = 0;
  pdFootRefPosZ( f ) = 0;
  pdFootRefAttX( f ) = 0;
  pdFootRefAttY( f ) = 0;
  pdFootRefAttZ( f ) = 0;
  if( dir == PD_FOOT_LEFT ){
    pdFootUWSign( pdFootUWPtr(f) ) = 1;
    pdFootZSign( pdFootZPtr(f) ) = 1;
  } else if( dir == PD_FOOT_RIGHT ){
    pdFootUWSign( pdFootUWPtr(f) ) = -1;
    pdFootZSign( pdFootZPtr(f) ) = -1;
  } else{
    ZRUNERROR( "invalid foot dir specified - %d", dir );
    pdFootDestroy( f );
    return;
  }
}

void pdFootDestroy(pdFoot *f)
{
  pdFootUWDestroy( pdFootUWPtr( f ) );
  pdFootZDestroy( pdFootZPtr( f ) );
  pdFootSetPos( f, 0, 0, 0 );
  pdFootSetDesPos( f, 0, 0, 0 );
  pdFootSetAtt( f, 0, 0, 0 );
  pdFootSetDesAtt( f, 0, 0, 0 );
  pdFootSetSR( f, NULL );
  pdFootCZPtr( f ) = NULL;
  pdFootResetTime( f );
  pdFootSetTimeStep( f, 0 );
  pdFootSetTrXK( f, 0 );
  pdFootSetTrXC( f, 0 );
  pdFootSetTrXOld( f, 0 );
  pdFootSetTrYK( f, 0 );
  pdFootSetTrYC( f, 0 );
  pdFootSetTrYOld( f, 0 );
  pdFootSetTrZK( f, 0 );
  pdFootSetTrZC( f, 0 );
  pdFootSetTrZOld( f, 0 );
  pdFootRefPosX( f ) = 0;
  pdFootRefPosY( f ) = 0;
  pdFootRefPosZ( f ) = 0;
  pdFootRefAttX( f ) = 0;
  pdFootRefAttY( f ) = 0;
  pdFootRefAttZ( f ) = 0;
}

void pdFootXformSRXYtoUW(pdFoot *f, zVec3DList *sr)
{
  zVec3D *p, *pp;
  zVec3DListCell *cp;
  zVec2D xy, uw;

  if( !( p = zAlloc( zVec3D, zListNum( sr ) ) ) ){
    ZALLOCERROR();
    zFree( p );
    exit( EXIT_FAILURE );
  }
  pp = p;
  zListForEach( sr, cp ){
    zVec2DCreate( xy, zVec3DElem(cp->data,zX), zVec3DElem(cp->data,zY) );
    pdFootXformXYtoUW( f, xy, uw );
    zVec3DCreate( pp++, uw[pdU], uw[pdW], zVec3DElem(cp->data,zZ) );
  }
  pdFootZSetSR( pdFootZPtr(f), p, zListNum(sr) );
  zFree( p );
}
