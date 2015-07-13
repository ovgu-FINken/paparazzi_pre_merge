#include "modules/eigen/eigen.h"
#include <Eigen/Core>

using namespace Eigen;

Matrix3i m;
Vector3i v;

void eigen_test_init(void){
  m = Matrix3i::Random();
  m = (m + Matrix3i::Constant(1)) * 50;
	v << 1,2,3;
}

void eigen_test_periodic(void){
	volatile Vector3i v2 = m*v;
}
