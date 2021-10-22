#include <iostream>

#include <boost/math/distributions/non_central_chi_squared.hpp>

int main(int argc, char *argv[])
{
    static constexpr int k = 3;
    static constexpr double lambda = 10;
    boost::math::non_central_chi_squared ncx2(k, lambda);

    std::cout << "ncx2 degrees of freedom, k: " << ncx2.degrees_of_freedom() << std::endl;
    std::cout << "ncx2 noncentrality, λ: " << ncx2.non_centrality() << std::endl;

    static constexpr int N = 100;
    static constexpr double step = 0.1;
    static constexpr double start = lambda - (N/2)*step;
    for (double x=start; x<(start+N*step); x += step) {
        std::cout << "f(" << x << ") = " << boost::math::pdf(ncx2, x) << std::endl;
    }

    return 0;
}