// Categorical Shape Registration
// Author: Florian Huber <s6flhube@uni-bonn.de>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#pragma once

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/vtk_lib_io.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include <shape_registration/cloud_manager.hpp>
#include <shape_registration/mesh_manager.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;

namespace categorical_registration
{

class PCLViewer
{

public:
	explicit PCLViewer ();
	~PCLViewer ();

	// Clouds
	void init(CloudManager* );

	void removeClouds();
	void removeTraCloud();
	void removeObsCloud();

	void updateClouds();
	void updateCanCloud();
	void updateTraCloud();
	void updateObsCloud();

	// Clouds visualization methods for the testing frame
	void removeCloudsTesting();
	void updateCloudsTesting();

	// Meshes
	void init(MeshManager* meshes);
	void setUpMeshes();
	void initTraMesh();
	void initObsMesh();
	void removeMeshes();
	void removeTraMesh();
	void removeObsMesh();

	void updateMeshes();
	void updateTraMesh();
	void updateObsMesh();

	// Clouds visualization methods for the testing frame
	void initMeshesTesting();
	void removeMeshesTesting();
	void updateMeshesTesting();

	void showCanonical(bool b)
	{ m_show_canonical = b; };
	void showObserved(bool b)
	{ m_show_observed = b; };
	void showDeformed(bool b)
	{ m_show_deformed = b; };

	void setSliderValue(const double &value)
	{ m_slider_value = value; };

	void spinOnce()
	{ m_viewer->spinOnce(); };

private:
	void setSizePoint();

	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;

	CloudManager* m_cloudData;
	MeshManager* m_meshData;

	bool m_show_canonical = true;
	bool m_show_observed = true;
	bool m_show_deformed = true;

	double m_slider_value = 1.0;

	const int m_point_size = 6;
};

}
