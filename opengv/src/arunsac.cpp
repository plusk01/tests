#include <iostream>
#include <vector>
#include <memory>

#include <Eigen/Dense>

#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opengv/point_cloud/methods.hpp>

/// \brief OpenGV adapters and problems
using Adapter3D = opengv::point_cloud::PointCloudAdapter;
using RansacProblem3D = opengv::sac_problems::point_cloud::PointCloudSacProblem;

int main()
{
    
	opengv::points_t landmarks1;
	landmarks1.emplace_back(-0.126249, -0.986089, 0.0836488);
	landmarks1.emplace_back(0.871028, 0.0386139, -0.0391561);
	landmarks1.emplace_back(0.090261, 0.0927056, -0.0139721);
	landmarks1.emplace_back(-0.039788, 0.338476, -0.112555);

	opengv::points_t landmarks2;
	landmarks2.emplace_back(0.422133,0.0453658,-0.0496704);
	landmarks2.emplace_back(-0.339228,0.930117,-0.0240017);
	landmarks2.emplace_back(0.219037,-0.0462751,1.11218);
	landmarks2.emplace_back(0.468469,0.122054,-0.0315951);

	Adapter3D adapter(landmarks1, landmarks2);

    std::cout << adapter.getNumberCorrespondences() << std::endl;

    opengv::sac::Ransac<RansacProblem3D> ransac;
    std::shared_ptr<RansacProblem3D> relposeproblem_ptr(new RansacProblem3D(adapter));
    ransac.sac_model_ = relposeproblem_ptr;
    ransac.threshold_ = 0.001;
    ransac.max_iterations_ = 50;
    ransac.computeModel(3);
//    auto& T = ransac.model_coefficients_;

//	std::cout << T.col(3).transpose() << std::endl;

    return 0;
}

