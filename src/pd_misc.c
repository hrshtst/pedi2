#include <pedi2/pd_misc.h>

/* tolerance to classify a vertex set as degenerate (all vertices on a
 * line or a point), which the convex-hull routine rejects */
#define PD_SUPPORT_LOOP_DEGENERACY_TOL (1.0e-6)

zLoop3D *pdSupportLoop(zVec3DData *vert, zLoop3D *loop)
{
  zVec3D *v, *pa, *pb, dir, rel, deviation;
  double d, dmax;

  zListInit( loop );
  if( zVec3DDataSize( vert ) == 0 ) return loop;
  /* the two mutually farthest vertices, in two sweeps */
  zVec3DDataRewind( vert );
  pa = pb = zVec3DDataFetch( vert );
  for( dmax=0, zVec3DDataRewind( vert ); ( v = zVec3DDataFetch( vert ) ); )
    if( ( d = zVec3DDist( v, pa ) ) > dmax ){ dmax = d; pb = v; }
  for( dmax=0, pa=pb, zVec3DDataRewind( vert ); ( v = zVec3DDataFetch( vert ) ); )
    if( ( d = zVec3DDist( v, pb ) ) > dmax ){ dmax = d; pa = v; }
  if( dmax < PD_SUPPORT_LOOP_DEGENERACY_TOL ) /* corner contact */
    return zLoop3DAdd( loop, pa ) ? loop : NULL;
  /* largest deviation of the set from the line through the extremes */
  zVec3DSub( pb, pa, &dir );
  zVec3DDivDRC( &dir, dmax );
  for( dmax=0, zVec3DDataRewind( vert ); ( v = zVec3DDataFetch( vert ) ); ){
    zVec3DSub( v, pa, &rel );
    zVec3DOuterProd( &rel, &dir, &deviation );
    if( ( d = zVec3DNorm( &deviation ) ) > dmax ) dmax = d;
  }
  if( dmax > PD_SUPPORT_LOOP_DEGENERACY_TOL )
    return zVec3DDataConvexHull2D( vert, loop );
  /* edge contact */
  if( zLoop3DAdd( loop, pa ) && zLoop3DAdd( loop, pb ) ) return loop;
  zLoop3DDestroy( loop );
  return NULL;
}
