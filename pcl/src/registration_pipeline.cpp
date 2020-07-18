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
#include <pcl/keypoints/harris_3d.h>
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

using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using PointCloudConstPtr = pcl::PointCloud<pcl::PointXYZ>::ConstPtr;

using SurfaceNormalsPtr = pcl::PointCloud<pcl::Normal>::Ptr;

// ----------------------------------------------------------------------------

static PointCloudPtr detectKeypointsHarris(const PointCloudConstPtr& points,
                                           const SurfaceNormalsPtr& normals,
                                           pcl::PointIndicesConstPtr& indices,
                                           double threshold, double radius)
{
  pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> detector;
  detector.setInputCloud(points);
  detector.setNormals(normals);
  detector.setNonMaxSupression(true);
  detector.setRefine(true);
  detector.setThreshold(threshold);
  detector.setRadius(radius);

  pcl::PointCloud<pcl::PointXYZI> keypoints_temp;
  detector.compute(keypoints_temp);

  PointCloudPtr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(keypoints_temp, *keypoints);

  indices = detector.getKeypointsIndices();

  return keypoints;
}

// ----------------------------------------------------------------------------

int main(int, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile("../data/g2b005_walls.pcd", *source_cloud);
    std::cout << "File 1 points: " << source_cloud->points.size() << std::endl;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    // Compute the normals
    pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud< pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(source_cloud);
    normalEstimation.setSearchMethod(tree);
    normalEstimation.setRadiusSearch(0.4);
    normalEstimation.compute(*source_normals);


    pcl::PointIndicesConstPtr keypoint_indices;
    auto keypoints = detectKeypointsHarris(source_cloud, source_normals, keypoint_indices, 5.0, 0.6);


    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(source_cloud);
    fpfh.setInputNormals(source_normals);
    fpfh.setIndices(keypoint_indices);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(0.2);
    fpfh.compute(*source_features);

    std::cout << "keypoints: " << keypoints->size() << std::endl;
    std::cout << "features: " << source_features->size() << std::endl;
    std::cout << "normals: " << source_normals->size() << std::endl;

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

    //
    // Visualization
    //

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    viewer.setBackgroundColor (0, 0, 0);
    viewer.resetCamera();

    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_source_cloud(source_cloud, 0.0, 0.0, 255);
    viewer.addPointCloud<pcl::PointXYZ>(source_cloud, /*handler_source_cloud,*/ "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(source_cloud, source_normals, 1, 0.7, "normals");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "normals");

    std::cout << "viewer" << std::endl;

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;

}