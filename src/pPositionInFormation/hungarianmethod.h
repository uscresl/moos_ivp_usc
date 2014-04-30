#ifndef HUNGARIANMETHOD_H_
#define HUNGARIANMETHOD_H_

// apt-get install libeigen3-dev
// may need to add symbolic link in /usr/include/ from eigen3/Eigen to Eigen
#include <Eigen/Core>

class HungarianMethod
{

#define HUNGARIAN_NOT_ASSIGNED 0
#define HUNGARIAN_ASSIGNED 1
#define INF (0x7FFFFFFF)

public:
  HungarianMethod();
  virtual ~HungarianMethod();

  /** This method computes the optimal assignment. **/
  static Eigen::VectorXi hungarian_solve(const Eigen::MatrixXd& c);

};

#endif /* HUNGARIANMETHOD_H_ */
