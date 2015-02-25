#include <pedi2/pd_vert.h>

void pdVertInit(pdVert *vert)
{
  vert->z = 0;
}

void pdVertUpdateZeta(pdVert *vert, double z)
{
  vert->z = z;
  vert->zeta = sqrt( RK_G / vert->z );
}
