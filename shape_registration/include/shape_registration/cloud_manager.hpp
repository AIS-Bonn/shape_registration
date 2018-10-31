// Categorical Shape Registration
// Author: Florian Huber <s6flhube@uni-bonn.de>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#pragma once

#include <shape_registration/pca.hpp>
#include <shape_registration/shape_utils.hpp>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// ROS
#include <ros/console.h>

// STL
#include <vector>

// Eigen
#include <Eigen/Dense>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::Normal PointNT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudNT;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;

namespace categorical_registration
{
class CloudManager
{

public:
	explicit CloudManager ();
	~CloudManager ();

	PointCloudT::Ptr getCanonical()
	{
		if (m_canonical_index > -1)
			return m_all_observed[m_canonical_index];
		else
		{
			ROS_WARN("The Canonical model has not been defined yet");
			return NULL;
		}
	};

	PointCloudT::Ptr getCurrentObserved()
	{
		if (m_canonical_index > -1)
			return m_all_observed[m_observed_index];
		else
		{
			ROS_WARN("The Observed model has not been defined yet");
			return NULL;
		}
	};

	PointCloudT::Ptr getCurrentTransformed()
	{ return m_all_transformed[m_observed_index]; };

	PointCloudT::Ptr getObservedAt(int idx)
	{ return m_all_observed[idx]; };

	PointCloudT::Ptr getTransformedAt(int idx)
	{ return m_all_transformed[idx]; };

	void setTransformedAt(int idx, const PointCloudT::Ptr cloud_in)
	{ m_all_transformed[idx] = cloud_in; };

	void addInstance(PointCloudT::Ptr cloud_in);

	void clearInstances();

	void deleteInstance(int idx);

	int getNumberInstances() const
	{ return m_all_observed.size(); }

	void initTransformedInstances();

	int getCanonicalIdx() const
	{ return m_canonical_index; };

	int getCurrentObservedIdx() const
	{ return m_observed_index; };

	void setCanonicalIdx(const int idx)
	{
		// When the canonical is changed all transformations have to be recomputed
		m_canonical_index = idx;
		setCpdDone(false);
		initTransformedInstances();
	};

	void setCurrentObservedIdx(const int idx)
	{
		if (m_all_observed.size() > 0)
			m_observed_index = idx;
		else
			ROS_INFO( "Please load PCDs before" );
	};

	PointCloudT::Ptr getObservedTesting()
	{ return m_observedPCD_testing; };

	PointCloudT::Ptr getTransformedTesting()
	{ return m_transformedPCD_testing; };

	void setObservedTesting(PointCloudT::Ptr in)
	{ m_observedPCD_testing = in; };

	void setObservedTesting(MatrixXd in)
	{ m_observedPCD_testing = matrixToCloud(in); };

	void setTransformedTesting(PointCloudT::Ptr in)
	{ m_transformedPCD_testing = in; };

	void setTransformedTesting(MatrixXd in)
	{ m_transformedPCD_testing = matrixToCloud(in); };

	MatrixXd getG() const
	{ return m_classG; };

	void setG(const MatrixXd G)
	{ m_classG = G; };

	MatrixXd getBigW() const
	{ return m_classW; };

	void setBigW(const MatrixXd W)
	{ m_classW = W; };

	std::vector<MatrixXd> getAllW() const
	{ return m_AllW; };

	void setAllW(const std::vector<MatrixXd> allW)
	{ m_AllW = allW; };
	
	bool getCpdDone()
	{ return m_cpdDone; }
	
	void setCpdDone(bool cpd)
	{ m_cpdDone = cpd; }
	
	pca::ptr getPca()
	{ return m_calculatedPCA; }

	void setPca(pca::ptr pca)
	{ m_calculatedPCA = pca; }

	bool getPcaDone()
	{ return m_pcaDone; }

	void setPcaDone(bool pca)
	{ m_pcaDone = pca; }

	bool getCloudFromFile(std::string file_name, PointCloudT::Ptr cloud, PointCloudNT::Ptr normals);
	
	static bool getCloudFromFile(std::string file_name, PointCloudT::Ptr cloud);

	// Save the matrix according to the canonical cloud
	MatrixXd m_canonicalMat;

private:
	// All observed clouds
	std::vector<PointCloudT::Ptr> m_all_observed;

	// All transformed clouds
	std::vector<PointCloudT::Ptr> m_all_transformed;

	// Observed and transformed PCDs for testing
	PointCloudT::Ptr m_observedPCD_testing = NULL;
	PointCloudT::Ptr m_transformedPCD_testing = NULL;

	// Numbers which of those are the canonical and observed instance
	int m_canonical_index = -1;
	int m_observed_index = -1;

	// Information from the CPD
	std::vector<MatrixXd> m_AllW;
	MatrixXd m_classW;
	MatrixXd m_classG;
	bool m_cpdDone = false;

	// PCA
	pca::ptr m_calculatedPCA;
	bool m_pcaDone = false;
};

}
