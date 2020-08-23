#pragma once

#include <string>
#include <fstream>
#include <filesystem>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class KittiReader {
public:
	KittiReader(const std::string& filename_prefix);
	void setSequence(const std::string& sequence);
	bool getNextPCD(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
	bool getNextPose(Eigen::Matrix4d& pose);
private:
	const std::string path_;
	std::string sequence_;
	std::filesystem::recursive_directory_iterator recursive_directory_iterator_;
	std::ifstream ifstream_pose_;
	std::vector<Eigen::Matrix4d> extrinsic_calibrations_;
};