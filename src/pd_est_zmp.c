#include <pedi2/pd_est_zmp.h>

void pdEstZMPInit(pdEstZMP *e_zmp)
{
  e_zmp->_t = 0;
  e_zmp->_dt = 0;
}

void pdEstZMPDestroy(pdEstZMP *e_zmp)
{
  pdEstZMPInit( e_zmp );
}
