// Categorical Shape Registration
// Author: Florian Huber <s6flhube@uni-bonn.de>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#include <shape_registration/pclviewer.h>

#include <pcl/io/pcd_io.h>

#include <ros/package.h>
#include <ros/console.h>

using namespace categorical_registration;
using namespace pcl::visualization;


PCLViewer::PCLViewer()
{
	m_viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	m_viewer->setBackgroundColor(1, 1, 1);
}


PCLViewer::~PCLViewer ()
{
}


//-------------------------------- Cloud methods ----------------------------------
//init with cloud input
void PCLViewer::init(CloudManager* clouds)
{
	m_cloudData = clouds;
}


void PCLViewer::removeClouds()
{
	removeObsCloud();
	removeTraCloud();
}


void PCLViewer::removeTraCloud()
{
	m_viewer->removePointCloud("transformed_cloud");
}


void PCLViewer::removeObsCloud()
{
	m_viewer->removePointCloud("observed_cloud");
}


void PCLViewer::updateClouds()
{
	updateCanCloud();
	updateObsCloud();
	updateTraCloud();
}


void PCLViewer::updateCanCloud()
{
	m_viewer->removePointCloud("canonical_cloud");

	if (m_show_canonical && m_cloudData->getCanonicalIdx() > -1)
	{
		colorCloud(m_cloudData->getCanonical(), 255, 0, 0);
		m_viewer->addPointCloud (m_cloudData->getCanonical(), "canonical_cloud");
		m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, m_point_size, "canonical_cloud");
	}
}


// Update the observed cloud
void PCLViewer::updateObsCloud()
{
	m_viewer->removePointCloud("observed_cloud");

	if ( m_show_observed && m_cloudData->getCurrentObservedIdx() > -1 )
	{
		m_viewer->addPointCloud (m_cloudData->getCurrentObserved(), "observed_cloud");
		m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, m_point_size, "observed_cloud");
	}
}


// Update the transformed cloud
void PCLViewer::updateTraCloud()
{
	m_viewer->removePointCloud("transformed_cloud");

	if ( m_show_deformed && m_cloudData->getCurrentObservedIdx() > -1 )
	{
		MatrixXd offset = cloudToMatrix(m_cloudData->getCurrentTransformed()) - cloudToMatrix(m_cloudData->getCanonical());
		MatrixXd tmp = cloudToMatrix(m_cloudData->getCanonical()) + m_slider_value * offset;

		PointCloudT::Ptr deformed = matrixToCloud(tmp);
		colorCloud(deformed, 0, 0, 255);

		m_viewer->addPointCloud (deformed, "transformed_cloud");
		m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, m_point_size, "transformed_cloud");
	}
}


// Remove the visualization from the testing frame
void PCLViewer::removeCloudsTesting()
{
	m_viewer->removePointCloud("canonical_cloud");
	m_viewer->removePointCloud("observed_cloud_testing");
	m_viewer->removePointCloud("transformed_cloud_testing");
}


// Update the visualization for the testing frame
void PCLViewer::updateCloudsTesting()
{
	removeCloudsTesting();

	if ( m_show_canonical && m_cloudData->getCanonicalIdx() > -1)
	{
		colorCloud(m_cloudData->getCanonical(), 255, 0, 0);
		m_viewer->addPointCloud (m_cloudData->getCanonical(), "canonical_cloud");
		m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, m_point_size, "canonical_cloud");
	}

	if (m_show_observed)
	{
		m_viewer->addPointCloud (m_cloudData->getObservedTesting(), "observed_cloud_testing");
		m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, m_point_size, "observed_cloud_testing");
	}

	if (m_show_deformed && m_cloudData->getCanonicalIdx() > -1 && m_cloudData->getPcaDone() )
	{
		MatrixXd offset = cloudToMatrix(m_cloudData->getTransformedTesting()) - cloudToMatrix(m_cloudData->getCanonical());
		MatrixXd tmp = cloudToMatrix(m_cloudData->getCanonical()) + m_slider_value * offset;

		PointCloudT::Ptr deformed = matrixToCloud(tmp);
		colorCloud(deformed, 0, 0, 255);

		m_viewer->addPointCloud (deformed, "transformed_cloud_testing");
		m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, m_point_size, "transformed_cloud_testing");
	}
}

//-----------------------------------Mesh mesthods-------------------------------------

void PCLViewer::init(MeshManager* meshes)
{
	m_meshData = meshes;
}


void PCLViewer::setUpMeshes()
{
	initObsMesh();
	initTraMesh();
}


void PCLViewer::initTraMesh()
{
	if (m_show_deformed && m_meshData->getCanonicalIdx() > -1)
		m_viewer->addModelFromPolyData (m_meshData->getCurrentTransformed(), "transformed_mesh");
	else
		ROS_INFO("The transformed mesh has not been defined.");
}


void PCLViewer::initObsMesh()
{
	if (m_show_observed)
		m_viewer->addModelFromPolyData (m_meshData->getCurrentObserved(), "observed_mesh");
}

// Remove the meshes
void PCLViewer::removeMeshes()
{
	removeObsMesh();
	removeTraMesh();
}


void PCLViewer::removeTraMesh()
{
	m_viewer->removeShape("transformed_mesh");
}


void PCLViewer::removeObsMesh()
{
	m_viewer->removeShape("observed_mesh");
}


// Update the meshes
void PCLViewer::updateMeshes()
{
	updateObsMesh();
	updateTraMesh();
}


// Update the transformed mesh
void PCLViewer::updateTraMesh()
{
	removeTraMesh();
	initTraMesh();
}


// Update the observed mesh
void PCLViewer::updateObsMesh()
{
	removeObsMesh();
	initObsMesh();
}


// Init the mesh visualization for the testing frame
void PCLViewer::initMeshesTesting()
{
	if (m_show_deformed)
		m_viewer->addModelFromPolyData (m_meshData->m_transformedPCAMesh, "pca_mesh");

	if (m_show_canonical)
	{
		colorMesh(m_meshData->getCanonical(), 255, 0, 0);
		m_viewer->addModelFromPolyData (m_meshData->getCanonical(), "canonical_mesh");
	}
}


// Remove the visualization from the testing frame
void PCLViewer::removeMeshesTesting()
{
	m_viewer->removeShape("canonical_mesh");
	m_viewer->removeShape("pca_mesh");
}


// Update the visualization for the testing frame
void PCLViewer::updateMeshesTesting()
{
	m_viewer->removeShape("pca_mesh");

	if (m_show_deformed)
	{
		colorMesh(m_meshData->m_transformedPCAMesh, 0, 0, 255);
		m_viewer->addModelFromPolyData (m_meshData->m_transformedPCAMesh, "pca_mesh");
	}
}
