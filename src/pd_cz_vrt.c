#include <pedi2/pd_cz_vrt.h>

void pdCZVrtInit(pdCZVrt *vrt)
{
  vrt->_zd = 0;
  vrt->_q1 = 0;
  vrt->_q2 = 0;

  vrt->reffz = 0;
  vrt->refacc = 0;
  vrt->refzmp = 0;
  vrt->zeta = 0;
}

void pdCZVrtDestroy(pdCZVrt *vrt)
{
  pdCZVrtInit( vrt );
}

static double _pdCZVrtXi(pdCZVrt *vrt);
static double _pdCZVrtK1(pdCZVrt *vrt);
static double _pdCZVrtK2(pdCZVrt *vrt);
static double _pdCZVrtBias(pdCZVrt *vrt);
static double _pdCZVrtCalcReactForce(pdCZVrt *vrt, double z, double vz);

double _pdCZVrtXi(pdCZVrt *vrt)
{
  return sqrt( RK_G / pdCZVrtRef(vrt) );
}

double _pdCZVrtK1(pdCZVrt *vrt)
{
  return zSqr(_pdCZVrtXi(vrt)) * pdCZVrtQ1(vrt) * pdCZVrtQ1(vrt);
}

double _pdCZVrtK2(pdCZVrt *vrt)
{
  return _pdCZVrtXi(vrt) * ( pdCZVrtQ1(vrt) + pdCZVrtQ1(vrt) );
}

double _pdCZVrtBias(pdCZVrt *vrt)
{
  return -RK_G;
}

double _pdCZVrtCalcReactForce(pdCZVrt *vrt, double z, double vz)
{
  return -_pdCZVrtK1(vrt) * ( z - pdCZVrtRef(vrt) ) - _pdCZVrtK2(vrt) * vz - _pdCZVrtBias(vrt);
}

double pdCZVrtCalcReactForce(pdCZVrt *vrt, double z, double vz)
{
  return zMax( 0, _pdCZVrtCalcReactForce( vrt, z, vz ) );
}

double pdCZVrtCalcAcc(pdCZVrt *vrt, double z, double vz)
{
  return pdCZVrtCalcReactForce( vrt, z, vz ) + _pdCZVrtBias( vrt );
}

double pdCZVrtCalcZMP(pdCZVrt *vrt)
{
  return 0;
}

double pdCZVrtCalcZeta(pdCZVrt *vrt, double z, double az, double zz )
{
  return sqrt( ( RK_G + az ) / ( z - zz ) );
}

void pdCZVrtUpdate(pdCZVrt *vrt, double z, double vz, double az, double zz)
{
  vrt->reffz = pdCZVrtCalcReactForce( vrt, z, vz );
  vrt->refacc = pdCZVrtCalcAcc( vrt, z, vz );
  vrt->refzmp = pdCZVrtCalcZMP( vrt );
  vrt->zeta = pdCZVrtCalcZeta( vrt, z, az, zz );
}
