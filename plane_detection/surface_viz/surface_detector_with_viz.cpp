#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

struct Rect {
  float min_x, max_x;
  float min_y, max_y;
};

bool isFlat(const pcl::ModelCoefficients::Ptr& coeffs) {
  Eigen::Vector3f n(coeffs->values[0], coeffs->values[1], coeffs->values[2]);
  double angle = std::acos(std::abs(n.dot(Eigen::Vector3f::UnitZ())) / n.norm()) * 180.0 / M_PI;
  return angle <= 15.0;
}

bool isNonOverlapping(const Rect& r, const std::vector<Rect>& existing) {
  for (const auto &o : existing) {
    bool overlap_x = !(r.max_x < o.min_x || r.min_x > o.max_x);
    bool overlap_y = !(r.max_y < o.min_y || r.min_y > o.max_y);
    if (overlap_x && overlap
      
      
      _y) return false;
  }
  return true;
}

CloudT::Ptr extractPoints(const CloudT::Ptr& cloud, const pcl::PointIndices::Ptr& inliers, bool negative) {
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(negative);
  auto out = std::make_shared<CloudT>();
  extract.filter(*out);
  return out;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <input.pcd>" << std::endl;
    return -1;
  }

  std::string pcd_file = argv[1];
  CloudT::Ptr cloud(new CloudT);
  if (pcl::io::loadPCDFile<PointT>(pcd_file, *cloud) == -1) {
    std::cerr << "Failed to load " << pcd_file << std::endl;
    return -1;
  }

  for (auto &pt : cloud->points) {
    pt.z = -pt.z;
  }

  CloudT::Ptr cloud_filtered(new CloudT(*cloud));
  std::vector<Rect> accepted_rects;
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Detected Surfaces"));
  viewer->addPointCloud(cloud, "original_cloud");
  int rect_id = 0;

  while (cloud_filtered->size() > 100) {
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coeffs);

    if (inliers->indices.size() < 500) break;
    if (!isFlat(coeffs)) {
      cloud_filtered = extractPoints(cloud_filtered, inliers, true);
      continue;
    }

    auto plane = extractPoints(cloud_filtered, inliers, false);
    auto proj = std::make_shared<CloudT>();
    pcl::ProjectInliers<PointT> projector;
    projector.setModelType(pcl::SACMODEL_PLANE);
    projector.setInputCloud(plane);
    projector.setModelCoefficients(coeffs);
    projector.filter(*proj);

    CloudT hull;
    pcl::ConvexHull<PointT> ch;
    ch.setInputCloud(proj);
    ch.setDimension(2);
    ch.reconstruct(hull);

    if (hull.size() < 3) {
      cloud_filtered = extractPoints(cloud_filtered, inliers, true);
      continue;
    }

    PointT min_pt, max_pt;
    pcl::getMinMax3D(hull, min_pt, max_pt);
    double length = max_pt.x - min_pt.x;
    double width = max_pt.y - min_pt.y;

    if ((length < 0.5 || length > 1.5) || (width < 0.5 || width > 1.5)) {
      cloud_filtered = extractPoints(cloud_filtered, inliers, true);
      continue;
    }

    Rect r{min_pt.x, max_pt.x, min_pt.y, max_pt.y};
    if (!isNonOverlapping(r, accepted_rects)) {
      cloud_filtered = extractPoints(cloud_filtered, inliers, true);
      continue;
    }

    accepted_rects.push_back(r);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*plane, centroid);
    std::cout << "Detected rectangle center: [" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << "]\n";

    std::string name = "rect_" + std::to_string(rect_id++);
    viewer->addPolygon<PointT>(hull.makeShared(), 1.0, 0.0, 0.0, name);
    viewer->addSphere(PointT(centroid[0], centroid[1], centroid[2]), 0.05, 0.0, 1.0, 0.0, name + "_center");

    cloud_filtered = extractPoints(cloud_filtered, inliers, true);
  }

  std::cout << "Detected " << rect_id << " valid rectangular flat surfaces.\n";
  viewer->spin();
  return 0;
}