#include <iostream>
#include <qpOASES.hpp>

int main()
{
  constexpr int n = 2;
  constexpr int m = 2;

  // problem data
  qpOASES::real_t H[n*n] = {2.0, 0.0, 0.0, 2.0};
  qpOASES::real_t g[n] = {-4.0, -4.0};
  qpOASES::real_t A[m*n]= {2.0, 4.0, 5.0, 5.0};
  qpOASES::real_t * lbA = nullptr; // no lower bound
  qpOASES::real_t ubA[m] = {28.0, 50.0};
  qpOASES::real_t lb[n] = {0.0, 0.0};
  qpOASES::real_t ub[n] = {8.0, 8.0};

  qpOASES::QProblem P(n, m);

  // solve QP
  int nWSR = 10; // max num "working set recalculations"
  P.init(H, g, A, lb, ub, lbA, ubA, nWSR);

  qpOASES::real_t xOpt[n];
  P.getPrimalSolution(xOpt);
  std::cout << "xOpt:" << std::endl;
  std::cout << "\t" << xOpt[0] << std::endl << "\t" << xOpt[1] << std::endl;
  std::cout << "objVal: " << P.getObjVal() << std::endl;

  return 0;
}
