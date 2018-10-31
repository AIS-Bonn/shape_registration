// Categorical Shape Registration
// Author: Florian Huber <s6flhube@uni-bonn.de>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#pragma once

#include <shape_registration/pca.hpp>

// VTK
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkTransform.h>
#include <vtkPerspectiveTransform.h>
#include <vtkGeneralTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkPLYReader.h>

// ROS
#include <ros/console.h>

// C++
#include <vector>

// Eigen
#include <Eigen/Dense>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;

namespace categorical_registration
{
class MeshManager
{

public:
	explicit MeshManager ();
	~MeshManager ();

	// Getters
	vtkSmartPointer<vtkPolyData> getCanonical()
	{ return m_all_observed[m_canonical_index]; };
	
	vtkSmartPointer<vtkPolyData> getCurrentObserved()
	{ return m_all_observed[m_observed_index]; };
	
	vtkSmartPointer<vtkPolyData> getCurrentTransformed()
	{ return m_all_transformed[m_observed_index]; };
	
	vtkSmartPointer<vtkPolyData> getObservedAt(int idx)
	{ return m_all_observed[idx]; };
	
	vtkSmartPointer<vtkPolyData> getTransformedAt(int idx)
	{ return m_all_transformed[idx]; };
	
	void setTransformedAt(int idx, const vtkSmartPointer<vtkPolyData> mesh_in)
	{ m_all_transformed[idx] = mesh_in; };

	void addInstance(vtkSmartPointer<vtkPolyData> mesh_in);

	void clearInstances();

	void deleteInstance(int idx);

	int getNumberInstances()
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
			ROS_INFO( "Please load meshes before" );
	};

	vtkSmartPointer<vtkPolyData> getObservedTesting()
	{ return m_observedMesh_testing; };

	vtkSmartPointer<vtkPolyData> getTransformedTesting()
	{ return m_transformedMesh_testing; };

	void setObservedTesting(vtkSmartPointer<vtkPolyData> in)
	{ m_observedMesh_testing = in; };

	void setTransformedTesting(vtkSmartPointer<vtkPolyData> in)
	{ m_transformedMesh_testing = in; };

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

	pca::ptr getPca()
	{ return m_calculatedPCA; }

	void setPca(pca::ptr pca)
	{ m_calculatedPCA = pca; }

	bool getPcaDone()
	{ return m_pcaDone;}

	void setPcaDone(bool pca)
	{ m_pcaDone = pca;}

	bool getCpdDone()
	{ return m_cpdDone;}

	void setCpdDone(bool cpd)
	{ m_cpdDone = cpd;}

	// Load a mesh from a .ply filde
	static bool getMeshFromFile(const std::string file_name, vtkSmartPointer<vtkPolyData> polydata);

	// Mesh to visualize the latent variables of the PCA
	vtkSmartPointer<vtkPolyData> m_transformedPCAMesh;

	// Save the matrix according to the canonical mesh after subsampling
	MatrixXd m_canonicalMat;

private:
	// All observed meshes
	std::vector<vtkSmartPointer<vtkPolyData>> m_all_observed;

	// All transformed meshes
	std::vector<vtkSmartPointer<vtkPolyData>> m_all_transformed;

	// Observed and transformed meshes for testing
	vtkSmartPointer<vtkPolyData> m_observedMesh_testing;
	vtkSmartPointer<vtkPolyData> m_transformedMesh_testing;

	// Numbers which of those are the canonical and observed instance
	int m_canonical_index = -1;
	int m_observed_index = -1;

	// Save the informations from the CPD
	std::vector<MatrixXd> m_AllW;
	MatrixXd m_classW;
	MatrixXd m_classG;
	bool m_cpdDone = false;

	// PCA
	pca::ptr m_calculatedPCA;
	bool m_pcaDone = false;

};

}
