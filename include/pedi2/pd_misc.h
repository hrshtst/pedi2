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

__END_DECLS

#endif /* __PD_MISC_H__ */
