#include <pedi2/pd_misc.h>

/* tolerance to classify a vertex set as degenerate (all vertices on a
 * line or a point), which the convex-hull routine rejects */
#define PD_SUPPORT_LOOP_DEGENERACY_TOL (1.0e-6)

/* convex hull of a vertex set in the x-y plane, whose vertices are
 * mapped back to the vertices of the set; the null pointer for a set on
 * a line in the x-y plane, leaving loop empty */
static zLoop3D *_pdSupportLoopHull(zVec3DData *vert, zLoop3D *loop)
{
  zVec2DData data;
  zVec2D *proj, *hv;
  zVec3D **src, *v;
  zPG2D ch;
  zLoop3D *ret = NULL;
  int n, i, j;

  n = zVec3DDataSize( vert );
  proj = zAlloc( zVec2D, n );
  src = zAlloc( zVec3D*, n );
  if( !proj || !src ){
    ZALLOCERROR();
    goto TERMINATE;
  }
  if( !zVec2DDataInitArray( &data, n ) ) goto TERMINATE;
  zVec3DDataRewind( vert );
  for( i=0; ( v = zVec3DDataFetch( vert ) ); i++ ){
    src[i] = v;
    zVec2DCreate( &proj[i], v->c.x, v->c.y );
    zVec2DDataAdd( &data, &proj[i] );
  }
  if( zVec2DDataConvexHull( &data, &ch ) ){
    for( i=0; i<zPG2DVertNum(&ch); i++ ){
      hv = zPG2DVert(&ch,i);
      for( j=0; j<n; j++ )
        if( proj[j].c.x == hv->c.x && proj[j].c.y == hv->c.y ) break;
      if( j == n || !zLoop3DAdd( loop, src[j] ) ){
        zLoop3DDestroy( loop );
        break;
      }
    }
    if( i == zPG2DVertNum(&ch) ) ret = loop;
    zPG2DDestroy( &ch );
  }
  zVec2DDataDestroy( &data );
 TERMINATE:
  zFree( proj );
  zFree( src );
  return ret;
}

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
  if( dmax > PD_SUPPORT_LOOP_DEGENERACY_TOL &&
      _pdSupportLoopHull( vert, loop ) ) return loop;
  /* edge contact, also of a set on a line in the x-y plane but spread
   * in height */
  if( zLoop3DAdd( loop, pa ) && zLoop3DAdd( loop, pb ) ) return loop;
  zLoop3DDestroy( loop );
  return NULL;
}

/* the cyclic successor of a cell of a loop */
static zLoop3DCell *_pdSupportLoopNext(zLoop3D *loop, zLoop3DCell *cell)
{
  return zListCellNext( cell ) == zListRoot( loop ) ?
    zListTail( loop ) : zListCellNext( cell );
}

/* closest point of a support region to a point in the x-y plane; the
 * orientation of the loop is the sign of its area, and every edge is
 * visited */
double pdSupportLoopClosest(zLoop3D *loop, zVec3D *p, zVec3D *cp)
{
  zLoop3DCell *c, *cn;
  double area = 0, ex, ey, len2, t, qx, qy, d, dmin = HUGE_VAL;
  bool inside;

  zVec3DCopy( p, cp );
  if( zListSize( loop ) == 0 ) return 0;
  /* twice the signed area */
  zListForEach( loop, c ){
    cn = _pdSupportLoopNext( loop, c );
    area += c->data->c.x * cn->data->c.y - cn->data->c.x * c->data->c.y;
  }
  inside = zListSize( loop ) >= 3 && area != 0;
  zListForEach( loop, c ){
    cn = _pdSupportLoopNext( loop, c );
    ex = cn->data->c.x - c->data->c.x;
    ey = cn->data->c.y - c->data->c.y;
    /* p is outside of the edge */
    if( ( area > 0 ? 1 : -1 ) *
        ( ex * ( p->c.y - c->data->c.y ) - ey * ( p->c.x - c->data->c.x ) ) < 0 )
      inside = false;
    /* the closest point on the edge */
    len2 = ex * ex + ey * ey;
    t = len2 > 0 ?
      ( ( p->c.x - c->data->c.x ) * ex + ( p->c.y - c->data->c.y ) * ey ) / len2 : 0;
    t = zLimit( t, 0, 1 );
    qx = c->data->c.x + t * ex;
    qy = c->data->c.y + t * ey;
    if( ( d = sqrt( zSqr( p->c.x - qx ) + zSqr( p->c.y - qy ) ) ) < dmin ){
      dmin = d;
      zVec3DCreate( cp, qx, qy, p->c.z );
    }
  }
  if( inside ){
    zVec3DCopy( p, cp );
    return 0;
  }
  return dmin;
}
