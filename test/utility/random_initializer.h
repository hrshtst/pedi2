#include <zeo/zeo_vec3d.h>
#include <random>

class RandomInitializer {
 public:
  RandomInitializer();
  RandomInitializer(double seed);
  ~RandomInitializer();

  double rand() { return mt_(); };
  double SetRandScalar(double &s);
  zVec3D &SetRandVec3D(zVec3D &v);
  zVec3D *SetRandVec3D(zVec3D *v);

 private:
  std::random_device rd_;
  std::mt19937 mt_;
};
