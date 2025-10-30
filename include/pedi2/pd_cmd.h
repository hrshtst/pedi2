#ifndef __PD_CMD_H__
#define __PD_CMD_H__

#include <zeo/zeo_vec3d.h>
#include <pedi2/pd_misc.h>

__BEGIN_DECLS

#define PD_CMD_ENTRY_NUM 39
typedef union{
  struct{
    double qu1, qu2;
    double qw1, qw2;
    double qz1, qz2;
    double kappa, rho, kr;
    double xd, yd, zd;
    double xdd, ydd, zdd;       /* user-commanded desired position (this should be removed) */
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

__PEDI2_EXPORT void pdCmdInit(pdCmd *cmd);
__PEDI2_EXPORT void pdCmdDefaultInit(pdCmd *cmd);
__PEDI2_EXPORT void pdCmdDestroy(pdCmd *cmd);

__PEDI2_EXPORT bool pdCmdTryStop(pdCmd *cmd);
__PEDI2_EXPORT bool pdCmdTryStep(pdCmd *cmd);
__PEDI2_EXPORT bool pdCmdTryWalk(pdCmd *cmd);
__PEDI2_EXPORT bool pdCmdTryWalkSideways(pdCmd *cmd);
__PEDI2_EXPORT bool pdCmdTryWarp(pdCmd *cmd);

__PEDI2_EXPORT void pdCmdDataFWrite(FILE *fp, pdCmd *cmd);
#define pdCmdDataWrite(c) pdCmdDataFWrite( stdout, c )
__PEDI2_EXPORT bool pdCmdDataFRead(FILE *fp, pdCmd *cmd);
#define pdCmdDataRead(c) pdCmdDataFRead( stdin, c )

__END_DECLS

#endif /* __PD_CMD_H__ */
