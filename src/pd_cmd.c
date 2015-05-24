#include <pedi2/pd_cmd.h>

void pdCmdInit(pdCmd *cmd)
{
  cmd->qu1 = cmd->qu2 = 0;
  cmd->qw1 = cmd->qw2 = 0;
  cmd->qz1 = cmd->qz2 = 0;
  cmd->kappa = 0;
  cmd->rho = 0;
  cmd->kr = 0;
  cmd->xd = cmd->yd = cmd->zd = 0;
  cmd->thetad = 0;
  cmd->vud = 0;
  cmd->vwd = 0;
  cmd->dist = 0;
  cmd->lfkx = cmd->lfky = cmd->lfkz = 0;
  cmd->lfcx = cmd->lfcy = cmd->lfcz = 0;
  cmd->rfkx = cmd->rfky = cmd->rfkz = 0;
  cmd->rfcx = cmd->rfcy = cmd->rfcz = 0;
  cmd->lfh = cmd->rfh = 0;
}

void pdCmdDefaultInit(pdCmd *cmd)
{
  pdCmdInit( cmd );
  cmd->qu1 = 1.0;
  cmd->qu2 = 0.0;
  cmd->qw1 = 1.0;
  cmd->qw2 = 1.5;
  cmd->qz1 = 1.0;
  cmd->qz2 = 0.5;
  cmd->rho = 0.0;
  cmd->kr = 1.0;
  cmd->thetad = -zPI_2;
  cmd->lfkx = cmd->lfky = cmd->lfkz = 3000;
  cmd->lfcx = cmd->lfcy = cmd->lfcz = 50;
  cmd->rfkx = cmd->rfky = cmd->rfkz = 3000;
  cmd->rfcx = cmd->rfcy = cmd->rfcz = 50;
}

void pdCmdDestroy(pdCmd *cmd)
{
  pdCmdInit( cmd );
}
