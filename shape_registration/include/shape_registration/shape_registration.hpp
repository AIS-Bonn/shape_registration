// Categorical Shape Registration
// Author: Florian Huber <s6flhube@uni-bonn.de>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#pragma once

#include <shape_registration/cloud_manager.hpp>
#include <shape_registration/mesh_manager.hpp>
#include <shape_registration/pclviewer.h>
#include <shape_registration/shape_registration.hpp>
#include <shape_registration/cpd.hpp>
#include <shape_registration/solver_thread.hpp>

// Qt
#ifndef Q_MOC_RUN
#include <QObject>
#endif

// C++
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace categorical_registration
{

class ShapeRegistration : public QObject
{
	Q_OBJECT

public:
	ShapeRegistration ();
	~ShapeRegistration ();

	void addTrainingInstance(const std::string filepath);

	void addTestingObserved(const std::string filepath);
	void setTestingObserved(PointCloudT::Ptr cloud);

	void setCanonicalMeshIdx(int idx);
	void setCanonicalPCDIdx(int idx);

	int getCanonicalMeshIdx() const
	{ return m_meshData.getCanonicalIdx(); };

	int getCanonicalPCDIdx() const
	{ return m_cloudData.getCanonicalIdx(); };

	int getMeshCount()
	{ return m_meshData.getNumberInstances(); };

	int getCloudCount()
	{ return m_cloudData.getNumberInstances(); };

	void setObservedMeshIdx(int idx);
	void setObservedPCDIdx(int idx);

	int getObservedMeshIdx()
	{ return m_meshData.getCurrentObservedIdx(); };
	int getObservedPCDIdx()
	{ return m_cloudData.getCurrentObservedIdx(); };

	void deleteMesh(const int idx);
	void deletePCD(const int idx);

	void updateViewer();

	// Calculate CPD on the instances stored at those numbers
	void calculateDeformationFields();
	void cpd(int can_number, int obs_number);
	void saveCPD();
	void loadCPD();

	// Methods used for PCA
	bool doPCA(int n_latent);
	void visualizePCA();
	MatrixXd pointsInLatentSpace();

	// Methods used for inference
	MatrixXd getModelFromLatent(const MatrixXd& latent, const Eigen::Affine3d& trans = Eigen::Affine3d::Identity());
	MatrixXd getModelFromLatent(const MatrixXd& latent, double *pose);

	void setUsingMeshes(bool b)
	{
		m_using_meshes = b;

		if (m_using_meshes && m_meshData.getNumberInstances() > 1)
		{
			m_viewer.removeClouds();
			m_viewer.setUpMeshes();
		}

		if (!m_using_meshes && m_cloudData.getNumberInstances() > 1)
		{
			m_viewer.removeMeshes();
			updateViewer();
		}
	};

	bool usingMeshes() const
	{ return m_using_meshes; };

	bool cpdDone() const
	{ return m_CPD_done; };

	bool cpdDoneOnMeshes()
	{ return m_meshData.getCpdDone(); };

	bool cpdDoneOnClouds()
	{ return m_cloudData.getCpdDone(); };

	std::string getCategoryFilepath()
	{ return m_category_filepath; };

	void setCategoryFilepath(std::string in)
	{ m_category_filepath = in; };

	PCLViewer* viewer()
	{ return &m_viewer; };

	//clear mesh and cloud data when the category is changed
	void clear();

	// Inference
	void fitToObserved();
	void cancelFitting();

	void setSlider(int value);

	// Variables to visualize the latent space after the PCA calculation
	void setLatentVariable1 (int value)
	{ m_latentVariable1 = value; };

	int getLatentVariable1 ()
	{ return m_latentVariable1; };

	void setLatentValue1 (double value)
	{ m_latentVisualization1 = value; };

	double getLatentValue1 ()
	{ return m_latentVisualization1; };

	void setLatentVariable2 (int value)
	{ m_latentVariable2 = value; };

	int getLatentVariable2 ()
	{ return m_latentVariable2; };

	void setLatentValue2 (double value)
	{ m_latentVisualization2 = value; };

	double getLatentValue2 ()
	{ return m_latentVisualization2; };

	void trainingView(bool b)
	{ m_training_view = b; };

private:
	void buildFromCPD();
	void buildCloudsFromCPD();
	void buildMeshesFromCPD();

	void cpdOnClouds(int can_number, int obs_number);
	void cpdOnMeshes(int can_number, int obs_number);

	// Methods used for PCA
	void buildBigW();
	MatrixXd wFromClassW(int index, MatrixXd bigW);

	MeshManager m_meshData;
	CloudManager m_cloudData;

	PCLViewer m_viewer;

	bool m_CPD_done = true;
	bool m_using_meshes = false;
	bool m_training_view = true;

	QReadWriteLock* m_mutex;

	solver_thread* m_solver_thread;

	// Config parameters
	const float m_config_beta = 1.0;
	const float m_config_lambda = 1.0;
	const int m_max_num_points_mesh = 500;

	//save the current category
	std::string m_category_filepath;

	// Variables to visualize the latent space after the PCA calculation
	int m_latentVariable1 = 0;
	double m_latentVisualization1 = 0;
	int m_latentVariable2 = 0;
	double m_latentVisualization2 = 0;

signals:
	void fit(MatrixXd observed_matrix, const MatrixXd& XStar, const Eigen::Affine3d& trans);
	void haltOpt();

public slots:
	void plotCallback();                                  // Called by the solver thread as it updates
	void modelCallback(double* XStar, double* pose);      // Called by the solver thread as it updates

private slots:
	void fitted(const MatrixXd& XStar, const Eigen::Affine3d& trans);
};

} // NS
