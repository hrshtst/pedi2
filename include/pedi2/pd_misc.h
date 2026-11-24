#ifndef __PD_MISC_H__
#define __PD_MISC_H__

#include <zeda/zeda.h>
#include <zm/zm.h>
#include <roki/rk_g.h>
#include <zeo/zeo_bv2d.h>
#include <pedi2/pedi2_export.h>

__BEGIN_DECLS

typedef byte pdAxis;
enum{
  pdU=0, pdW
};

/* support loop of a vertex set: the planar convex hull of a set
 * spread on a plane, the two extreme vertices of a set on a line
 * (edge contact), or the single vertex of a set on a point (corner
 * contact), which the convex-hull routine rejects.  Any previous
 * content of loop is dropped without being freed, so destroy it
 * beforehand; an empty set leaves loop empty.  It returns the null
 * pointer if it fails, leaving loop empty. */
__PEDI2_EXPORT zLoop3D *pdSupportLoop(zVec3DData *vert, zLoop3D *loop);

/* closest point of a support region to a point p in the x-y plane:
 * p itself if p is inside of the region enclosed by a convex loop,
 * otherwise the nearest point on its edges, put into cp at the height
 * of p.  The vertices may differ in height, and the loop may start at
 * any vertex, run in either direction and have vertices on its sides;
 * a loop of one or two vertices is a point or a segment, and an empty
 * loop leaves p as it is.  It returns the distance from p to the
 * region in the x-y plane, which is zero inside. */
__PEDI2_EXPORT double pdSupportLoopClosest(zLoop3D *loop, zVec3D *p, zVec3D *cp);

__END_DECLS

#endif /* __PD_MISC_H__ */
