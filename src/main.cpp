////////////////////////////////////////////////////////////////////////////
//  Copyright 2018-2019 Gong Zheng, 
//  Xiamen University. All Rights Reserved.
//
//  For more information see <https://github.com/AlienCat-K/LiDAR-Automatic-Calibration>
//  If you use this code, please cite the corresponding publications as 
//  listed on the above website.
//
//  Permission to use, copy, modify and distribute this software and its
//  documentation for educational, research and non-profit purposes only.
//  Any modification based on this work must be open source and prohibited
//  for commercial use.
//  You must retain, in the source form of any derivative works that you 
//  distribute, all copyright, patent, trademark, and attribution notices 
//  from the source form of this work.
//   
//
////////////////////////////////////////////////////////////////////////////

#include "fitline.h"
#include "ransac.h"
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <iostream>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/gicp.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_utils/Transform3.h>
#include <PointCloudMapper.h>

using namespace std;
namespace gu = geometry_utils;

#define PI (3.1415926535897932346f)

char framesDir[100] = "../data/L-LiDAR-Frames";

std::string itos(int i)
{
	std::stringstream s;
	s << i;
	return s.str();
}

int main()
{
	//================== Step.1 Reading H-LiDAR map data =====================//

	std::cout << "Reading H-LiDAR map data..." << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr H_LiDAR_Map(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/H-LiDAR-Map-data/H_LiDAR_Map.pcd", *H_LiDAR_Map) == -1)
	{
		PCL_ERROR("Couldn't read H_LiDAR_Map \n");
		return (-1);
	}
	std::cout << "Loaded " << H_LiDAR_Map->size() << " data points from H_LiDAR_Map.pcd" << std::endl;

	//put it into map
	PointCloudMapper maps;
	maps.Initialize();
	pcl::PointCloud<pcl::PointXYZ>::Ptr unused(new pcl::PointCloud<pcl::PointXYZ>);
	maps.InsertPoints(H_LiDAR_Map, unused.get());

	//================== Step.2 Reading H-LiDAR's Trajectory and init guess=====================//

	ifstream T_Mat_File("../data/T_Matrix.txt");
	Eigen::Matrix4f T_Matrix = Eigen::Matrix4f::Identity();

	ifstream initFile("../data/Init_Matrix.txt");

	Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

	for (int mat_i = 0; mat_i != 4; mat_i++)
	{
		for (int mat_j = 0; mat_j != 4; mat_j++)
		{
			initFile >> init_guess(mat_i, mat_j);
		}
	}

	//================== Step.3 Reading L-LiDAR frames =====================//

	struct dirent **namelist;
	int framenumbers = scandir(framesDir, &namelist, 0, alphasort) - 2;
	int frame_count = 0;
	cout << "Loaded " << framenumbers << " frames from L-LiDAR" << endl;

	//=================================
	//prepare ICP
	pcl::PointCloud<pcl::PointXYZ>::Ptr ICP_output_cloud(new pcl::PointCloud<pcl::PointXYZ>); //not use,but necessary
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setTransformationEpsilon(0.0000000001); //0.0000000001
	icp.setMaxCorrespondenceDistance(10);
	icp.setMaximumIterations(35);
	icp.setRANSACIterations(0);
	icp.setMaximumOptimizerIterations(50); // default 20
	//=================================
	//              START
	//=================================
	while (frame_count < framenumbers)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr frames(new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(string(framesDir) + "/" + itos(frame_count) + ".pcd", *frames) == -1)
		{
			PCL_ERROR("Couldn't read H_LiDAR_Map \n");
			return (-1);
		}
		//std::cout << "Loaded " << frames->size() << " data points from frames" << std::endl;

		//Load H-LiDAR's Trajectory
		for (int mat_i = 0; mat_i != 4; mat_i++)
		{
			for (int mat_j = 0; mat_j != 4; mat_j++)
			{
				T_Mat_File >> T_Matrix(mat_i, mat_j);
			}
		}
		//std::cout << T_Matrix.matrix() << std::endl;
		//================== Step.4 Start calibration =====================//

		pcl::PointCloud<pcl::PointXYZ>::Ptr trans_output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr final_output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*frames, *trans_output_cloud, init_guess); //Tiny_T * init_guess   ->update this matrix
		pcl::transformPointCloud(*trans_output_cloud, *final_output_cloud, T_Matrix);

		pcl::PointCloud<pcl::PointXYZ>::Ptr neighbors_L(new pcl::PointCloud<pcl::PointXYZ>); //201 neighbors points from nap202
		maps.ApproxNearestNeighbors(*final_output_cloud, neighbors_L.get());

		//INVERSE T_mat==============================
		// why can not use inverse_mat.inverse()?
		gu::Transform3 inverse_mat;
		inverse_mat.translation = gu::Vec3(T_Matrix(0, 3), T_Matrix(1, 3), T_Matrix(2, 3));
		inverse_mat.rotation = gu::Rot3(T_Matrix(0, 0), T_Matrix(0, 1), T_Matrix(0, 2),
										T_Matrix(1, 0), T_Matrix(1, 1), T_Matrix(1, 2),
										T_Matrix(2, 0), T_Matrix(2, 1), T_Matrix(2, 2));

		const gu::Transform3 estimate = gu::PoseInverse(inverse_mat); //integrated_estimate from config parameters
		const Eigen::Matrix<double, 3, 3> T_Matrix_Inverse_R = estimate.rotation.Eigen();
		const Eigen::Matrix<double, 3, 1> T_Matrix_Inverse_T = estimate.translation.Eigen();

		Eigen::Matrix4d T_Matrix_Inverse;
		T_Matrix_Inverse.block(0, 0, 3, 3) = T_Matrix_Inverse_R;
		T_Matrix_Inverse.block(0, 3, 3, 1) = T_Matrix_Inverse_T;

		//====== Core step ======//
		//==============================
		pcl::PointCloud<pcl::PointXYZ>::Ptr neighbors_trans(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*neighbors_L, *neighbors_trans, T_Matrix_Inverse);

		//Do ICP and get the tiny trans T
		icp.setInputSource(trans_output_cloud); //201
		icp.setInputTarget(neighbors_trans);	//202 (201's neighbor's point cloud)
		icp.align(*ICP_output_cloud);
		const Eigen::Matrix4f Tiny_T = icp.getFinalTransformation();

		std::cout << "Score: " << icp.getFitnessScore() << std::endl;
		

		Eigen::Matrix4f Final_Calib_T = Eigen::Matrix4f::Identity();

		Final_Calib_T = Tiny_T * init_guess;
		std::cout << Final_Calib_T.matrix() << std::endl;
		cout<<Final_Calib_T.matrix()<<endl;
		init_guess = Final_Calib_T;

		frame_count++;
	}

	return 0;
}
