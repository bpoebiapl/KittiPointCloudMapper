#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include "KittiReader.h"


int main() {
	const double leaf_size = 0.05;
	KittiReader reader("F:/dataset/data_odometry_velodyne/dataset");
	reader.setSequence("01");
	pcl::visualization::PCLVisualizer viewer;

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_acc = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	std::set<std::array<int64_t, 3>> set;

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
	while (reader.getNextPCD(cloud)) {

		Eigen::Matrix4d m;
		if (reader.getNextPose(m)) {
			pcl::transformPointCloud<pcl::PointXYZI>(*cloud, *cloud, m.cast<float>());
			for (uint64_t point_i = 0; point_i < cloud->size(); point_i++) {
				const pcl::PointXYZI& point = cloud->points[point_i];
				const std::array<int64_t, 3> voxel = { static_cast<int64_t>(std::floor(point.x / leaf_size)), static_cast<int64_t>(std::floor(point.y / leaf_size)), static_cast<int64_t>(std::floor(point.z / leaf_size)) };
				if (!set.insert(voxel).second) continue;
				cloud_acc->push_back(cloud->points[point_i]);
			}
			pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> c_handler(cloud_acc, "intensity");
			if (!viewer.updatePointCloud(cloud_acc, c_handler, "cloud")) {
				viewer.addPointCloud(cloud_acc, c_handler, "cloud");
			}
			viewer.spinOnce();
		}
	}
	viewer.spin();
}