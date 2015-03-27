#include <pedi2/pd_cz.h>

void pdCZZMPPhase(pdCZ *c, double dw, double vw, zComplex *pz)
{
  /* zComplexCreate( pz, pdCZZMPRad(c) - dw, -(pdCZPrmRad(c)->q1*pdCZPrmRad(c)->q2+1)*vw/(pdCZZeta(c)*sqrt(pdCZPrmRad(c)->q1*pdCZPrmRad(c)->q2)) ); */
  zComplexCreate( pz, pdCZZMPRad(c), -(pdCZPrmRad(c)->q1*pdCZPrmRad(c)->q2+1)*vw/(pdCZZeta(c)*sqrt(pdCZPrmRad(c)->q1*pdCZPrmRad(c)->q2)) );
}
