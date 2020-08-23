#include "KittiReader.h"

#include <fstream>
#include <regex>

#ifdef _DEBUG
#include <iostream>
#endif

KittiReader::KittiReader(const std::string& path) : path_(path) {
}

void KittiReader::setSequence(const std::string& sequence) {
	sequence_ = sequence;
	std::filesystem::path p(path_ + "/sequences/" + sequence_ + "/velodyne/");
	recursive_directory_iterator_ = std::filesystem::recursive_directory_iterator(p);
	ifstream_pose_.close();
	ifstream_pose_.open(path_ + "/poses/" + sequence_ + ".txt");
	std::ifstream ifstream(path_ + "/sequences/" + sequence_ + "/calib.txt");
	for(int cal_i = 0; cal_i < 5; cal_i++){
		std::string line;
		std::getline(ifstream, line);
		std::smatch match;
		std::regex_search(line, match, std::regex("..: (\\S+) (\\S+) (\\S+) (\\S+) (\\S+) (\\S+) (\\S+) (\\S+) (\\S+) (\\S+) (\\S+) (\\S+)"));
		int i = 1;
		Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
		for (int row = 0; row < 3; row++) {
			for (int col = 0; col < 4; col++) {
				std::stringstream stream(match[i++].str());
				stream >> m(row, col);
			}
		}
		extrinsic_calibrations_.push_back(m);
	}
}

bool KittiReader::getNextPCD(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
	if (recursive_directory_iterator_ == std::filesystem::recursive_directory_iterator()) return false;
	const std::string filename = recursive_directory_iterator_->path().generic_string();
#ifdef _DEBUG
	std::cout << filename << std::endl;
#endif
	std::ifstream input(filename, std::ios::in | std::ios::binary);
	if (!input.good()) {
#ifdef _DEBUG
		std::cerr << "Could not read file: " << filename << std::endl;
#endif
		return false;
	}
	input.seekg(0, std::ios::beg);

	cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

	pcl::PointXYZI point;
	while (input.good() && !input.eof()) {
		input.read((char*)&point.x, 3 * sizeof(float));
		input.read((char*)&point.intensity, sizeof(float));
		cloud->push_back(point);
	}
	input.close();

#ifdef _DEBUG
	std::cout << "Read KTTI point cloud with " << cloud->size() << std::endl;
#endif

	recursive_directory_iterator_++;
	return true;
}
bool KittiReader::getNextPose(Eigen::Matrix4d& pose) {
	if (!ifstream_pose_.good()) {
#ifdef _DEBUG
		std::cerr << "Could not read file: " << path_ + "/poses/" + sequence_ + ".txt" << std::endl;
#endif
		return false;
	}
	std::string line;
	std::getline(ifstream_pose_, line);
	std::stringstream stringstream(line);
	pose.setIdentity();
	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 4; col++) {
			double value = std::numeric_limits<double>::max();
			stringstream >> value;
			if (value == std::numeric_limits<double>::max()) return false;
			pose(row, col) = value;
		}
	}
	pose = pose * extrinsic_calibrations_[4];
	return true;
}