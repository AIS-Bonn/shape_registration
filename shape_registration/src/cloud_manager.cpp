// Categorical Shape registration
// Author: Florian Huber <s6flhube@uni-bonn.de>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#include <shape_registration/cloud_manager.hpp>

using namespace categorical_registration;


CloudManager::CloudManager()
{
	m_transformedPCD_testing.reset(new pcl::PointCloud<PointT> ());
	m_observedPCD_testing.reset(new pcl::PointCloud<PointT> ());
}


CloudManager::~CloudManager ()
{
}


void CloudManager::addInstance(PointCloudT::Ptr cloud_in)
{
	m_all_observed.push_back(cloud_in);

	// The first added instance serve as the canonical instance
	if ( getNumberInstances() == 1 )
		m_canonical_index = 0;

	PointCloudT::Ptr copy_cloud;
	copy_cloud.reset(new pcl::PointCloud<PointT> ());

	// DeepCopy
	copy_cloud->points.resize(getCanonical()->points.size());

	for (unsigned int i = 0; i < getCanonical()->points.size(); i++)
	{
		copy_cloud->points[i].x = getCanonical()->points[i].x;
		copy_cloud->points[i].y = getCanonical()->points[i].y;
		copy_cloud->points[i].z = getCanonical()->points[i].z;
	}

	colorCloud(copy_cloud, 0, 255, 0);

	m_all_transformed.push_back(copy_cloud);
}


void CloudManager::deleteInstance(int idx)
{
	if (getNumberInstances() == 1)
	{
		clearInstances();
		m_canonical_index = -1;
		m_observed_index = -1;
	}
	else if (getNumberInstances() > 0)
	{
		if (idx < m_canonical_index || m_canonical_index + 1 == (int)getNumberInstances())
		{
			setCanonicalIdx(m_canonical_index - 1);
			ROS_INFO( "Canonical index changed  = %d", m_canonical_index );
		}

		if (idx < m_observed_index || m_observed_index + 1 == (int)getNumberInstances())
		{
			setCurrentObservedIdx(m_observed_index - 1);
			ROS_INFO( "Observed_index changed = %d", m_observed_index );
		}

		ROS_INFO( "Entry %d was deleted", idx );

		m_all_observed.erase(m_all_observed.begin() + idx);
		m_all_transformed.erase(m_all_transformed.begin() + idx);
	}
}


void CloudManager::clearInstances()
{
	m_all_observed.clear();
	m_all_transformed.clear();
};


void CloudManager::initTransformedInstances()
{
	for (int i = 0; i < getNumberInstances(); i++)
	{
		PointCloudT::Ptr copy_cloud;
		copy_cloud.reset(new pcl::PointCloud<PointT> ());
		pcl::copyPointCloud (*getCanonical(), *copy_cloud);
		m_all_transformed[i] = copy_cloud;
		colorCloud(m_all_transformed[i], 0, 0, 255);
	}
}


bool CloudManager::getCloudFromFile(std::string file_name, pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointNT>::Ptr normals)
{
	if (!(file_name.length() > 0))
	{
		ROS_ERROR("Invalid Filename to Load");
		return false;
	}

	// Temporary cloud to load the files
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr loadCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(file_name, *loadCloud) == -1) 
	{
		ROS_ERROR ("Couldn't read the .pcd file \n");
		return false;
	}

	ROS_INFO_STREAM( "Loaded " << loadCloud->size() << " data points from " << file_name << std::endl);

	// Split informations about cloud and normals
	cloud->points.resize(loadCloud->points.size());
	normals->points.resize(loadCloud->points.size());

	// DeepCopy
	for (unsigned int i = 0; i < loadCloud->points.size(); i++)
	{
		cloud->points[i].x = loadCloud->points[i].x;
		cloud->points[i].y = loadCloud->points[i].y;
		cloud->points[i].z = loadCloud->points[i].z;

		normals->points[i].normal_x = loadCloud->points[i].normal_x;
		normals->points[i].normal_y = loadCloud->points[i].normal_y;
		normals->points[i].normal_z = loadCloud->points[i].normal_z;
	}

	return true;
}


bool CloudManager::getCloudFromFile(std::string file_name, pcl::PointCloud<PointT>::Ptr cloud)
{
	if (!(file_name.length() > 0))
	{
		ROS_ERROR("Invalid Filename to Load");
		return false;
	}

	// Temporary cloud to load the files
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr loadCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(file_name, *loadCloud) == -1)
	{
		ROS_ERROR ("Couldn't read the .pcd file \n");
		return false;
	}

	ROS_INFO_STREAM( "Loaded " << loadCloud->size() << " data points from " << file_name << std::endl);

	cloud->points.resize(loadCloud->points.size());

	// DeepCopy
	for (unsigned int i = 0; i < loadCloud->points.size(); i++)
	{
		cloud->points[i].x = loadCloud->points[i].x;
		cloud->points[i].y = loadCloud->points[i].y;
		cloud->points[i].z = loadCloud->points[i].z;
	}

	return true;
}

