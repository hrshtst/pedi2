#ifndef __PD_CMD_H__
#define __PD_CMD_H__

#include <zeo/zeo_vec3d.h>

__BEGIN_DECLS

typedef struct{
  double qu1, qu2;
  double qw1, qw2;
  double qz1, qz2;
  double kappa, rho, kr;
  double xd, yd, zd;
  double thetad;
  double vud, vwd, dist;
  double lfkx, lfky, lfkz;
  double lfcx, lfcy, lfcz;
  double rfkx, rfky, rfkz;
  double rfcx, rfcy, rfcz;
  double lfh, rfh;
} pdCmd;

__EXPORT void pdCmdInit(pdCmd *cmd);
__EXPORT void pdCmdDefaultInit(pdCmd *cmd);
__EXPORT void pdCmdDestroy(pdCmd *cmd);

__END_DECLS

#endif /* __PD_CMD_H__ */
