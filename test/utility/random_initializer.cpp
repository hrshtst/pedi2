#include "random_initializer.h"

RandomInitializer::RandomInitializer()
    : rd_(), mt_(rd_())
{}

RandomInitializer::RandomInitializer(double seed)
    : rd_(), mt_(seed)
{}

RandomInitializer::~RandomInitializer()
{}

double RandomInitializer::SetRandScalar(double &s)
{
  s = mt_();
  return s;
}

zVec3D &RandomInitializer::SetRandVec3D(zVec3D &v)
{
  for(int i=0; i<3; i++){
    v.e[i] = mt_();
  }
  return v;
}
