#include <iostream>
#include <vector>

#include <Eigen/Core>

// From the man himself,
// https://stackoverflow.com/a/50039760/2392520

template<typename Func>
struct lambda_as_visitor_wrapper : Func {
    lambda_as_visitor_wrapper(const Func& f) : Func(f) {}
    template<typename S,typename I>
    void init(const S& v, I i, I j) { return Func::operator()(v,i,j); }
};

template<typename Mat, typename Func>
void visit_lambda(const Mat& m, const Func& f)
{
    lambda_as_visitor_wrapper<Func> visitor(f);
    m.visit(visitor);
}

int main() {
  int n = 5;
  double th = 0.5;
  Eigen::MatrixXd M = Eigen::MatrixXd::Random(n,n);

  std::vector<int> ii, jj;
  visit_lambda(M,
    [&ii,&jj,th](double v, int i, int j) {
      if (v>th) {
        ii.push_back(i);
        jj.push_back(j);

        std::cout << i << "," << j << std::endl;
      }
    });

  std::cout << M << "\n\n";

  for (size_t i=0; i<ii.size(); ++i) {
    std::cout << "(" << ii[i] << "," << jj[i] << ") ";
  }
  std::cout << std::endl;

  // std::vector<int> ind{0,1};
  // std::cout << M(Eigen::all,ind) << std::endl;
  // std::cout << M(ii,jj) << std::endl;
  // Apparently not until 3.4: https://eigen.tuxfamily.org/dox-devel/group__TutorialSlicingIndexing.html

  size_t itrr = 0, itrc = 0;
  Eigen::MatrixXd subM(M.rows(), M.cols());

  size_t curi = 0, curj = 0;

  for (size_t k=0; k<ii.size(); ++k) {
    
    if (k == 0) {
      curi = ii[k];
      curj = jj[k];
    }

    if (ii[k] > curi) {
      itrr++;
      curi = ii[k];
    }
    

    if (jj[k] > curj) {
      itrc++;
      itrr = 0;
      curj = jj[k];
    }

    subM(itrr,itrc) = M(ii[k], jj[k]);

  }

  std::cout << subM << std::endl;

 // for (size_t i=0; i<ii.size(); ++i) {
 //   for (size_t j=0; j<jj.size(); ++j) {
 //     subM(i,j) = M(ii[i],jj[i]);
 //   }
 // }

  return 0;

}
