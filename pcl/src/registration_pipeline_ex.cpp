/**
 * @file registration_pipeline.cpp
 * @brief Point cloud registration pipeline
 * @author Parker Lusk <plusk@mit.edu>
 * @date 13 July 2020
 */

// see https://stackoverflow.com/questions/41105987/pcl-feature-matching

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/features/fpfh.h>


using namespace pcl;
using namespace pcl::io;

pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
pcl::visualization::PCLVisualizer ICPView("ICP Viewer");

double computeCloudResolution(const pcl::PointCloud<PointXYZ>::ConstPtr &cloud)
{
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> sqr_distances(2);
    pcl::search::KdTree<PointXYZ> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (!pcl_isfinite((*cloud)[i].x))
        {
            continue;
        }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
    if (nres == 2)
    {
        res += sqrt(sqr_distances[1]);
        ++n_points;
    }
  }
  if (n_points != 0)
  {
      res /= n_points;
  }
  return res;
}


int main(int, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new PointCloud<PointXYZ>());
    loadPCDFile("../data/g2b005_walls.pcd", *source_cloud);
    std::cout << "File 1 points: " << source_cloud->points.size() << std::endl;
    //file 1

        // Compute model_resolution

    double model_resolution = computeCloudResolution(source_cloud);

    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    iss_detector.setSearchMethod(tree);
    iss_detector.setSalientRadius(10 * model_resolution);
    iss_detector.setNonMaxRadius(8 * model_resolution);
    iss_detector.setThreshold21(0.2);
    iss_detector.setThreshold32(0.2);
    iss_detector.setMinNeighbors(10);
    iss_detector.setNumberOfThreads(10);
    iss_detector.setInputCloud(source_cloud);
    iss_detector.compute((*source_keypoints));
    pcl::PointIndicesConstPtr keypoints_indices = iss_detector.getKeypointsIndices();


    std::cout << "No of ISS points in the result are " << (*source_keypoints).points.size() << std::endl;
    std::string Name = "g2b005_walls_ISSKP.pcd";
    savePCDFileASCII(Name, (*source_keypoints));


    // Compute the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(source_cloud);
    normalEstimation.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud< pcl::Normal>);
    normalEstimation.setRadiusSearch(0.2);
    normalEstimation.compute(*source_normals);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(source_cloud);
    fpfh.setInputNormals(source_normals);
    // fpfh.setIndices(keypoints_indices);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(0.2);
    fpfh.compute(*source_features);

    /* SHOT optional Descriptor
    pcl::PointCloud<pcl::SHOT352>::Ptr source_features(new pcl::PointCloud<pcl::SHOT352>());
    pcl::SHOTEstimation< pcl::PointXYZ, pcl::Normal, pcl::SHOT352 > shot;
    shot.setSearchMethod(tree); //kdtree
    shot.setIndices(keypoints_indices); //keypoints
    shot.setInputCloud(source_cloud); //input
    shot.setInputNormals(source_normals); //normals
    shot.setRadiusSearch(0.2); //support
    shot.compute(*source_features); //descriptors
    */


//Target Point Cloud

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new PointCloud<PointXYZ>());
    loadPCDFile("../data/g2b006_walls.pcd", *target_cloud);
    std::cout << "File 2 points: " << target_cloud->points.size() << std::endl;

    // Compute model_resolution

    double model_resolution_1 = computeCloudResolution(target_cloud);

    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector_1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints(new pcl::PointCloud<pcl::PointXYZ>());

    iss_detector_1.setSearchMethod(tree);
    iss_detector_1.setSalientRadius(10 * model_resolution_1);
    iss_detector_1.setNonMaxRadius(8 * model_resolution_1);
    iss_detector_1.setThreshold21(0.2);
    iss_detector_1.setThreshold32(0.2);
    iss_detector_1.setMinNeighbors(10);
    iss_detector_1.setNumberOfThreads(10);
    iss_detector_1.setInputCloud(target_cloud);
    iss_detector_1.compute((*target_keypoints));
    pcl::PointIndicesConstPtr keypoints_indices_1 = iss_detector_1.getKeypointsIndices();


    std::cout << "No of ISS points in the result are " << (*target_keypoints).points.size() << std::endl;
    savePCDFileASCII("g2b006_walls_ISSKP.pcd", (*target_keypoints));


    // Compute the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation_1;
    normalEstimation_1.setInputCloud(target_cloud);
    normalEstimation_1.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud< pcl::Normal>);
    normalEstimation_1.setRadiusSearch(0.2);
    normalEstimation_1.compute(*target_normals);


    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh1;
    fpfh1.setInputCloud(target_cloud);
    fpfh1.setInputNormals(target_normals);
    // fpfh1.setIndices(keypoints_indices_1);
    fpfh1.setSearchMethod(tree);
    fpfh1.setRadiusSearch(0.2);
    fpfh1.compute(*target_features);

    /* Shot Optional
    pcl::PointCloud<pcl::SHOT352>::Ptr target_features(new pcl::PointCloud<pcl::SHOT352>());
    pcl::SHOTEstimation< pcl::PointXYZ, pcl::Normal, pcl::SHOT352 > shot_1;
    shot_1.setSearchMethod(tree); //kdtree
    shot_1.setIndices(keypoints_indices_1); //keypoints
    shot_1.setInputCloud(target_cloud); //input
    shot_1.setInputNormals(target_normals); //normals
    shot_1.setRadiusSearch(0.2); //support
    shot_1.compute(*target_features); //descriptors
    */


// estimate correspondences
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    est.setInputSource(source_features);
    est.setInputTarget(target_features);
    est.determineCorrespondences(*correspondences);

    // Duplication rejection Duplicate

    pcl::CorrespondencesPtr correspondences_result_rej_one_to_one(new pcl::Correspondences);
    pcl::registration::CorrespondenceRejectorOneToOne corr_rej_one_to_one;
    corr_rej_one_to_one.setInputCorrespondences(correspondences);
    corr_rej_one_to_one.getCorrespondences(*correspondences_result_rej_one_to_one);


    // Correspondance rejection RANSAC

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector_sac;
    pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences);
    rejector_sac.setInputSource(source_cloud);
    rejector_sac.setInputTarget(target_cloud);
    rejector_sac.setInlierThreshold(2.5); // distance in m, not the squared distance
    rejector_sac.setMaximumIterations(1000000);
    rejector_sac.setRefineModel(false);
    rejector_sac.setInputCorrespondences(correspondences_result_rej_one_to_one);;
    rejector_sac.getCorrespondences(*correspondences_filtered);
    correspondences.swap(correspondences_filtered);
    std::cout << correspondences->size() << " vs. " << correspondences_filtered->size() << std::endl;
    transform = rejector_sac.getBestTransformation();   // Transformation Estimation method 1


    // Transformation Estimation method 2
    //pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;
    //transformation_estimation.estimateRigidTransformation(*source_keypoints, *target_keypoints, *correspondences, transform);
    std::cout << "Estimated Transform:" << std::endl << transform << std::endl;

    // / refinement transform source using transformation matrix ///////////////////////////////////////////////////////

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source_cloud, *transformed_source, transform);
    savePCDFileASCII("Transformed.pcd", (*transformed_source));


    // viewer.setBackgroundColor (0, 0, 0);
    viewer.setBackgroundColor(1, 1, 1);
    viewer.resetCamera();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_source_cloud(transformed_source, 150, 80, 80);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_source_keypoints(source_keypoints, 255, 0, 0);

    viewer.addPointCloud<pcl::PointXYZ>(transformed_source, handler_source_cloud, "source_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source_cloud");
    // viewer.addPointCloud<pcl::PointXYZ>(source_keypoints, handler_source_keypoints, "source_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_target_cloud(target_cloud, 80, 150, 80);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_target_keypoints(target_keypoints, 0, 255, 0);

    viewer.addPointCloud<pcl::PointXYZ>(target_cloud, handler_target_cloud, "target_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud");
    // viewer.addPointCloud<pcl::PointXYZ>(target_keypoints, handler_target_keypoints, "target_keypoints");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "target_keypoints");
    // viewer.addCorrespondences<pcl::PointXYZ>(source_keypoints, target_keypoints, *correspondences, 1, "correspondences");
    // viewer.addCorrespondences<pcl::PointXYZ>(source_cloud, target_cloud, *correspondences, 1, "correspondences");

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(transformed_source);
    icp.setInputTarget(target_cloud);
    icp.align(*final_output);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;


    ICPView.addPointCloud<pcl::PointXYZ>(final_output, handler_source_cloud, "Final_cloud");
    // ICPView.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "source_keypoints");
    while (!viewer.wasStopped())
    {

        viewer.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    while (!ICPView.wasStopped())
    {

        ICPView.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    /*
    // Setup the SHOT features
    typedef pcl::SHOT352 ShotFeature;
    pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, ShotFeature> shotEstimation;

    shotEstimation.setInputCloud(model);
    shotEstimation.setInputNormals(normals);
    shotEstimation.setIndices(keypoint_indices);

    // Use the same KdTree from the normal estimation
    shotEstimation.setSearchMethod(tree);
    pcl::PointCloud<ShotFeature>::Ptr shotFeatures(new pcl::PointCloud<ShotFeature>);
    //spinImageEstimation.setRadiusSearch (0.2);
    shotEstimation.setKSearch(10);

    // Actually compute the spin images
    shotEstimation.compute(*shotFeatures);
    std::cout << "SHOT output points.size (): " << shotFeatures->points.size() << std::endl;

    // Display and retrieve the SHOT descriptor for the first point.
    ShotFeature descriptor = shotFeatures->points[0];
    std::cout << descriptor << std::endl;
    */

    return 0;

}