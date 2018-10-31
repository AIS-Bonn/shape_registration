// Categorical Non-rigid registration
// Author: Florian Huber <s6flhube@uni-bonn.de>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#pragma once

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

//VTK
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include <fstream>

#include <shape_registration/cpd.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;


namespace categorical_registration
{
	// Methods for Pointclouds
	MatrixXd cloudToMatrix(PointCloudT::Ptr);
	PointCloudT::Ptr matrixToCloud(MatrixXd mat, 
								const unsigned int &r = 255, 
								const unsigned int &g = 0, 
								const unsigned int &b = 0);
	PointCloudT::Ptr colorCloud(PointCloudT::Ptr cloud, 
								const unsigned int &r = 255, 
								const unsigned int &g = 0, 
								const unsigned int &b = 0);
	PointCloudT::Ptr transformCloud(PointCloudT::Ptr cloud_in, Eigen::Affine3d transform);

	// Methods for meshes
	MatrixXd meshToMatrix(vtkSmartPointer<vtkPolyData> mesh);
	vtkSmartPointer<vtkPolyData> matrixToMesh(MatrixXd mat, 
											vtkSmartPointer<vtkPolyData> meshTop, 
											const unsigned int &r = 255, 
											const unsigned int &g = 0, 
											const unsigned int &b = 0);
	vtkSmartPointer<vtkPolyData> matrixToMesh(MatrixXd mat, 
											const unsigned int &r = 255, 
											const unsigned int &g = 0, 
										   const unsigned int &b = 0);

	vtkSmartPointer<vtkPolyData> colorMesh(vtkSmartPointer<vtkPolyData> mesh, 
										const unsigned int &r, 
										const unsigned int &g, 
										const unsigned int &b);
	vtkSmartPointer<vtkPolyData> subSample(vtkSmartPointer<vtkPolyData> mesh, double percentage = 0.5);
	vtkSmartPointer<vtkPolyData> quadricClustering(vtkSmartPointer<vtkPolyData> mesh, double factor = 32);
	vtkSmartPointer<vtkPolyData> resolution(vtkSmartPointer<vtkPolyData> mesh, double factor = 2);
	vtkSmartPointer<vtkPolyData> extractOutside(vtkSmartPointer<vtkPolyData> mesh);

} //NS
