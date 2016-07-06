#ifndef __PD_CMD_H__
#define __PD_CMD_H__

#include <zeo/zeo_vec3d.h>

__BEGIN_DECLS

#define PD_CMD_ENTRY_NUM 36
typedef union{
  struct{
    double qu1, qu2;
    double qw1, qw2;
    double qz1, qz2;
    double kappa, rho, kr;
    double xd, yd, zd;
    double thetad;
    double vud, vwd, dist;
    double kx, ky, kz;
    double bx, by, bz;
    double lfkx, lfky, lfkz;
    double lfcx, lfcy, lfcz;
    double rfkx, rfky, rfkz;
    double rfcx, rfcy, rfcz;
    double lfh, rfh;
  };
  double entry[PD_CMD_ENTRY_NUM];
} pdCmd;

__EXPORT void pdCmdInit(pdCmd *cmd);
__EXPORT void pdCmdDefaultInit(pdCmd *cmd);
__EXPORT void pdCmdDestroy(pdCmd *cmd);

__EXPORT bool pdCmdTryStop(pdCmd *cmd);
__EXPORT bool pdCmdTryStep(pdCmd *cmd);
__EXPORT bool pdCmdTryWalk(pdCmd *cmd);
__EXPORT bool pdCmdTryWalkSideways(pdCmd *cmd);

__EXPORT void pdCmdDataFWrite(FILE *fp, pdCmd *cmd);
#define pdCmdDataWrite(c) pdCmdDataFWrite( stdout, c )
__EXPORT bool pdCmdDataFRead(FILE *fp, pdCmd *cmd);
#define pdCmdDataRead(c) pdCmdDataFRead( stdin, c )

__END_DECLS

#endif /* __PD_CMD_H__ */
