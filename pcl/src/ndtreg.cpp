/**
 * @file registration_pipeline.cpp
 * @brief Point cloud registration pipeline
 * @author Parker Lusk <plusk@mit.edu>
 * @date 13 July 2020
 */

// see https://stackoverflow.com/questions/41105987/pcl-feature-matching
// see https://github.com/Autoware-AI/utilities/blob/master/multi_lidar_calibrator/src/multi_lidar_calibrator.cpp

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/cloud_viewer.h>

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;
using PointCloudConstPtr = pcl::PointCloud<PointT>::ConstPtr;

using SurfaceNormalsPtr = pcl::PointCloud<pcl::Normal>::Ptr;

// ----------------------------------------------------------------------------

static void DownsampleCloud(PointCloudConstPtr in_cloud_ptr,
                            PointCloudPtr out_cloud_ptr, double in_leaf_size)
{
  pcl::VoxelGrid<PointT> voxelized;
  voxelized.setInputCloud(in_cloud_ptr);
  voxelized.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
  voxelized.filter(*out_cloud_ptr);
}

// ----------------------------------------------------------------------------

int main(int argc, char const *argv[])
{

  //
  // Load model (tgt) and data (src) clouds
  //

  PointCloudPtr model(new PointCloud);
  pcl::io::loadPCDFile("../data/g2b005_walls.pcd", *model);
  std::cout << "Model points: " << model->points.size() << std::endl;

  PointCloudPtr data(new PointCloud);
  pcl::io::loadPCDFile("../data/g2b006_walls_oct5_2d_fullfov.pcd", *data);
  std::cout << "Data points: " << data->points.size() << std::endl;

  //
  // Prior transformation
  //

  // initial transformation guess
  Eigen::Translation3f t0(-6.0, 30.0, 0.0);
  Eigen::AngleAxisf Rx0(0.0, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf Ry0(0.0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf Rz0(-3.14159/2.0, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f T0 = (t0 * Rx0 * Ry0 * Rz0).matrix();

  pcl::transformPointCloud(*data, *data, T0);

  //
  // Downsample data cloud
  //

  PointCloudPtr data_filtered(new PointCloud);

  static constexpr float VOXELSIZE = 0.5;
  DownsampleCloud(data, data_filtered, VOXELSIZE);

  //
  // Initialize Normal Distributions Transform (NDT)
  //
  
  std::cout << "here" << std::endl;

  pcl::NormalDistributionsTransform<PointT, PointT> ndt;
  ndt.setTransformationEpsilon(0.01);
  ndt.setStepSize(0.1);
  ndt.setResolution(0.1);
  ndt.setMaximumIterations(400);
  ndt.setInputSource(data);
  ndt.setInputTarget(model);
  std::cout << "there" << std::endl;

  // find the transformation using ndt + prior
  PointCloudPtr aligned(new PointCloud);
  ndt.align(*aligned);
  std::cout << "everywhere" << std::endl;

  std::cout << "Normal Distributions Transform converged:" << ndt.hasConverged()
            << " score: " << ndt.getFitnessScore() << " prob:" << ndt.getTransformationProbability() << std::endl;

  // Transform the unfiltered cloud
  pcl::transformPointCloud(*data, *aligned, ndt.getFinalTransformation());

  //
  // Visualization
  //

  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

  viewer.setBackgroundColor(0, 0, 0);
  viewer.resetCamera();

  viewer.addPointCloud<PointT>(model, "model");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "model");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model");

  viewer.addPointCloud<PointT>(data, "data");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "data");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "data");

  viewer.addPointCloud<PointT>(aligned, "aligned");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 1, "aligned");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "aligned");




  pcl::visualization::PCLVisualizer viewer2("Cloud Viewer");

  viewer2.setBackgroundColor(0, 0, 0);
  viewer2.resetCamera();

  viewer2.addPointCloud<PointT>(data_filtered, "data_filtered");
  viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "data_filtered");
  viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "data_filtered");



  while (!viewer.wasStopped() && !viewer2.wasStopped()) {
    viewer.spinOnce();
    viewer2.spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
