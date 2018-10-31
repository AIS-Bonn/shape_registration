// Categorical Shape Registration
// Author: Florian Huber <s6flhube@uni-bonn.de>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#include <shape_registration/mesh_manager.hpp>
#include <shape_registration/shape_registration.hpp>

using namespace categorical_registration;

MeshManager::MeshManager()
{
	// Initialize the mesh for testing
	m_transformedPCAMesh = vtkSmartPointer<vtkPolyData>::New();
}


MeshManager::~MeshManager()
{
}


void MeshManager::addInstance(vtkSmartPointer<vtkPolyData> mesh_in)
{
	m_all_observed.push_back(mesh_in);

	// The first added instance serve initially as the canonical instance
	if ( getNumberInstances() == 1 )
		m_canonical_index = 0;

	vtkSmartPointer<vtkPolyData> copy_mesh = vtkSmartPointer<vtkPolyData>::New();
	copy_mesh->DeepCopy(getCanonical());
	colorMesh(copy_mesh, 0, 255, 0);

	m_all_transformed.push_back(copy_mesh);
}


void MeshManager::clearInstances()
{
	m_all_observed.clear();
	m_all_transformed.clear();
};


void MeshManager::deleteInstance(int idx)
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


void MeshManager::initTransformedInstances()
{
	for (int i = 0; i < getNumberInstances(); i++)
	{
		m_all_transformed[i]->DeepCopy( getCanonical() );
		colorMesh(m_all_transformed[i], 0, 0, 255);
	}
}


bool MeshManager::getMeshFromFile(const std::string file_name, vtkSmartPointer<vtkPolyData> polydata)
{
	if ( file_name.empty() )
	{
		ROS_ERROR("Invalid Filename to Load");
		return false;
	}

	vtkSmartPointer<vtkPLYReader> readerQuery = vtkSmartPointer<vtkPLYReader>::New ();
	readerQuery->SetFileName (file_name.c_str());
	readerQuery->Update();
	polydata->DeepCopy(readerQuery->GetOutput() );

	ROS_INFO_STREAM("loaded mesh from " << file_name );

	return true;
}

