// Categorical Shape Registration
// Author: Florian Huber <s6flhube@uni-bonn.de>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#include <shape_registration/shape_registration.hpp>

#include <ros/package.h>

using namespace categorical_registration;

ShapeRegistration::ShapeRegistration()
{
	m_viewer.init(&m_meshData);
	m_viewer.init(&m_cloudData);

	m_mutex = new QReadWriteLock();

	m_solver_thread = new solver_thread(m_mutex);

	QObject::connect(this, SIGNAL(fit(MatrixXd, const MatrixXd&, const Eigen::Affine3d&)),
	                 m_solver_thread, SLOT(fit(MatrixXd, const MatrixXd&, const Eigen::Affine3d&)));
	QObject::connect(this, SIGNAL(haltOpt()), m_solver_thread, SLOT(halt()));
	QObject::connect(m_solver_thread, SIGNAL(fitted(const MatrixXd&, const Eigen::Affine3d&)),
	                 this, SLOT(fitted(const MatrixXd&, const Eigen::Affine3d&)));

	QObject::connect(m_solver_thread->getCallback(), SIGNAL(plotCallback()), this, SLOT(plotCallback()));
	QObject::connect(m_solver_thread->getCallback(), SIGNAL(modelCallback(double*, double*)), this, SLOT(modelCallback(double*, double*)));
}


ShapeRegistration::~ShapeRegistration ()
{
	m_solver_thread->halt();
	m_solver_thread->exit();

	delete m_solver_thread;
}


void ShapeRegistration::addTrainingInstance(const std::string filepath)
{
	if (filepath.substr(filepath.find_last_of(".") + 1) == "ply")
	// Meshes are loaded
	{
		vtkSmartPointer<vtkPolyData> tmp_mesh = vtkSmartPointer<vtkPolyData>::New();
		MeshManager::getMeshFromFile(filepath.c_str(), tmp_mesh);
		colorMesh(tmp_mesh, 0, 255, 0);

		m_meshData.addInstance(tmp_mesh);

		if (m_meshData.getNumberInstances() == 2)
		{
			m_meshData.setCurrentObservedIdx(1);

			if ( usingMeshes() )
				m_viewer.updateMeshes();
		}
	}
	else if (filepath.substr(filepath.find_last_of(".") + 1) == "pcd")
	// Clouds are loaded
	{
		PointCloudT::Ptr tmp_cloud;
		tmp_cloud.reset(new pcl::PointCloud<PointT> ());
		CloudManager::getCloudFromFile(filepath.c_str(), tmp_cloud);
		colorCloud(tmp_cloud, 0, 255, 0);

		m_cloudData.addInstance(tmp_cloud);

		if (m_cloudData.getNumberInstances() == 2)
		{
			m_cloudData.setCurrentObservedIdx(1);

			if ( !usingMeshes() )
				m_viewer.updateClouds();
		}
	}
}


void ShapeRegistration::addTestingObserved(const std::string filepath)
{
	if (filepath.substr(filepath.find_last_of(".") + 1) == "ply")
	// Meshes are loaded
	{
		vtkSmartPointer<vtkPolyData> tmp_mesh = vtkSmartPointer<vtkPolyData>::New();
		MeshManager::getMeshFromFile(filepath.c_str(), tmp_mesh);
		colorMesh(tmp_mesh, 0, 255, 0);

		//m_meshData.setObservedTesting(tmp_mesh);

		m_viewer.updateMeshesTesting();
	}
	else if (filepath.substr(filepath.find_last_of(".") + 1) == "pcd")
	// Clouds are loaded
	{
		PointCloudT::Ptr tmp_cloud( new pcl::PointCloud<PointT> () );
		CloudManager::getCloudFromFile(filepath.c_str(), tmp_cloud);
		colorCloud(tmp_cloud, 0, 255, 0);

		m_cloudData.setObservedTesting(tmp_cloud);

		m_viewer.updateCloudsTesting();
	}
}


void ShapeRegistration::setTestingObserved(PointCloudT::Ptr cloud)
{
	m_cloudData.setObservedTesting(cloud);
	m_viewer.updateCloudsTesting();
}


void ShapeRegistration::setCanonicalMeshIdx(int idx)
{
	m_meshData.setCanonicalIdx(idx);
	m_viewer.updateMeshes();
}


void ShapeRegistration::setCanonicalPCDIdx(int idx)
{
	m_cloudData.setCanonicalIdx(idx);
	m_viewer.updateClouds();
}


void ShapeRegistration::setObservedMeshIdx(int idx)
{
	m_meshData.setCurrentObservedIdx(idx);
	m_viewer.updateMeshes();
}


void ShapeRegistration::setObservedPCDIdx(int idx)
{
	m_cloudData.setCurrentObservedIdx(idx);
	m_viewer.updateClouds();
}


void ShapeRegistration::deleteMesh(const int idx)
{
	m_meshData.deleteInstance(idx);
	m_viewer.updateMeshes();
}


void ShapeRegistration::deletePCD(const int idx)
{
	m_cloudData.deleteInstance(idx);
	m_viewer.updateClouds();
}


void ShapeRegistration::updateViewer()
{
	if (m_training_view)
	{
		if ( m_using_meshes)
			m_viewer.updateMeshes();
		else
			m_viewer.updateClouds();
	}
	else
	{
		if ( m_using_meshes)
			m_viewer.updateMeshes();
		else
			m_viewer.updateCloudsTesting();
	}
}


void ShapeRegistration::calculateDeformationFields()
{
	m_CPD_done = false;

	if ( m_using_meshes)
	{
		if ( m_meshData.getNumberInstances() < 2)
		{
			ROS_INFO( "Please load at least 2 meshes before" );
		}
		else if ( m_meshData.getCpdDone() )
		{
			ROS_INFO( "CPD has been already calculated" );
		}
		else
		{
			for (int i = 0; i < (int) m_meshData.getNumberInstances(); i++)
			{
				cpd( m_meshData.getCanonicalIdx(), i);
			}

			m_viewer.updateMeshes();
		}
	}
	else
	{
		if (m_cloudData.getNumberInstances() < 2)
		{
			ROS_INFO( "Please load at least 2 clouds before" );
		}
		else if ( m_cloudData.getCpdDone() )
		{
			ROS_INFO( "CPD is already calculated" );
		}
		else
		{
			for (int i = 0; i < (int) m_cloudData.getNumberInstances(); i++)
			{
				cpd( m_cloudData.getCanonicalIdx(), i);
			}

			m_viewer.updateClouds();
		}
	}

	m_CPD_done = true;
}


void ShapeRegistration::cpd(int can_number, int obs_number)
{
	if (m_using_meshes)
	{
		cpdOnMeshes(can_number, obs_number);
	}
	else
	{
		cpdOnClouds(can_number, obs_number);
	}
}


// Perform CPD on point clouds
void ShapeRegistration::cpdOnClouds(int can_number, int obs_number)
{
	// Just to inform about the progress
	int calculationNumber = obs_number + 1;

	if (obs_number > can_number)
	{
		calculationNumber --;
	}

	MatrixXd observed_mat;
	MatrixXd canonical_mat;
	MatrixXd erg_mat;
	MatrixXd W;
	cpdG::NonrigidResult result;

	// Just calculate for the non canonical instances
	if (can_number != obs_number)
	{
		ROS_INFO_STREAM( "Calculating number " << calculationNumber << " from " << m_cloudData.getNumberInstances() - 1 );
		observed_mat = cloudToMatrix(m_cloudData.getObservedAt(obs_number));
		canonical_mat = cloudToMatrix(m_cloudData.getCanonical());

		// Perform CPD
		result = doCPD(observed_mat, canonical_mat,  m_config_beta,  m_config_lambda);
		erg_mat = result.points;
		W = result.getW();

		// Save the transformed cloud
		m_cloudData.setTransformedAt(obs_number, matrixToCloud(erg_mat));

		// Save the canonical matrix and G once
		if ( !m_cloudData.getCpdDone() )
		{
			m_cloudData.setG (result.G);
			m_cloudData.m_canonicalMat = canonical_mat;

			// Save an instance for visualizing PCA after the CPD
			m_cloudData.setTransformedTesting (matrixToCloud(m_cloudData.m_canonicalMat, 0, 0 , 255) );
		}

		m_cloudData.setCpdDone(true);
		ROS_INFO_STREAM( "\nCalculation " << calculationNumber << " from " << m_cloudData.getNumberInstances() - 1 << " is done." );
	}
	//if canonical == observed
	else
	{
		// Save some W to make the numbering consistent
		W = MatrixXd::Zero(1, 1);

		// Save the subsampled canonical instance as the transformed
		erg_mat = cloudToMatrix(m_cloudData.getObservedAt(can_number));
		m_cloudData.setTransformedAt(obs_number, m_cloudData.getCanonical()) ;
	}

	// Save the W for the PCA
	auto allW = m_cloudData.getAllW();

	while ( (int)allW.size() < (obs_number + 1))
	{
		MatrixXd A(1, 1);
		A << 0;
		allW.push_back(A);
	}

	allW[obs_number] = W;
	m_cloudData.setAllW(allW);
}


// Perform CPD on meshes
void ShapeRegistration::cpdOnMeshes(int can_number, int obs_number)
{
	// Just to inform about the progress
	int calculationNumber = obs_number + 1;

	if (obs_number > can_number)
	{
		calculationNumber --;
	}

	MatrixXd observed_mat;
	MatrixXd canonical_mat;
	MatrixXd erg_mat;
	MatrixXd W;
	cpdG::NonrigidResult result;

	// Just calculate for the non canonical instances
	if (can_number != obs_number)
	{
		ROS_INFO_STREAM( "Calculating number " << calculationNumber << " from " << m_meshData.getNumberInstances() - 1 );

		vtkSmartPointer<vtkPolyData> subObs = vtkSmartPointer<vtkPolyData>::New();
		subObs->DeepCopy(m_meshData.getObservedAt(obs_number));

		vtkSmartPointer<vtkPolyData> subCan = vtkSmartPointer<vtkPolyData>::New();
		subCan->DeepCopy(m_meshData.getCanonical() );

		// Add points to the mesh to make it more dense
		subObs = resolution(subObs, 1);
		subCan = resolution(subCan, 1);

		// Subsample the mesh for faster calculation
		while (subObs->GetNumberOfPoints() > m_max_num_points_mesh)
		{
			subObs = quadricClustering(subObs, 32);
		}

		while (subCan->GetNumberOfPoints() > m_max_num_points_mesh)
		{
			subCan = quadricClustering(subCan, 32);
		}

		// Take the points of the mesh and store them in a matrix
		observed_mat = meshToMatrix(subObs);
		canonical_mat = meshToMatrix(subCan);

		// Perform CPD
		result = doCPD(observed_mat, canonical_mat,  m_config_beta,  m_config_lambda);
		erg_mat = result.points;
		W = result.getW();

		// Save the transformed mesh
		m_meshData.setTransformedAt(obs_number, matrixToMesh(erg_mat, subCan));

		// Save the canonical matrix and G once
		if ( !m_meshData.getCpdDone() )
		{
			m_meshData.setG(result.G);
			m_meshData.m_canonicalMat = canonical_mat;

			// Save an instance for visualizing PCA after the CPD
			m_meshData.m_transformedPCAMesh = matrixToMesh(m_meshData.m_canonicalMat, m_meshData.getTransformedAt(obs_number), 0, 0 , 255);
		}

		m_meshData.setCpdDone(true);

		ROS_INFO_STREAM( "\nCalculation " << calculationNumber << " from " << m_meshData.getNumberInstances() - 1 << " is done." );
	}
	else
	{
		// Save some W to make the numbering consistent
		W = MatrixXd::Zero(1, 1);

		// Save the subsampled canonical instance as the transformed
		erg_mat = meshToMatrix(m_meshData.getObservedAt(can_number));
		m_meshData.setTransformedAt(obs_number, m_meshData.getCanonical()) ;
	}

	// Save the W for the PCA
	auto allW = m_meshData.getAllW();

	while ( (int)allW.size() < (obs_number + 1))
	{
		MatrixXd A(1, 1);
		A << 0;
		allW.push_back(A);
	}

	allW[obs_number] = W;

	m_meshData.setAllW(allW);
}


bool ShapeRegistration::doPCA(int n_latent)
{
	if ((m_using_meshes && !m_meshData.getCpdDone() ) || ( !m_using_meshes && !m_cloudData.getCpdDone() ) )
	{
		ROS_INFO_STREAM( "Please calculate CPD first." );
		return false;
	}

	buildBigW();

	// Calculate PCA
	pca::ptr pca = pca::New();
	pca::PCA_METHOD pcaMethod = pca::PCA_EM;

	if (m_using_meshes)
	{
		if ( m_meshData.getCpdDone() )
		{
			pca->calculatePCA(m_meshData.getBigW(), n_latent, pcaMethod);
			m_meshData.setPca(pca);
			m_meshData.setPcaDone(true);
			return true;
		}
		else
		{
			ROS_INFO( "Please load or calculate CPD first" );
			return false;
		}
	}
	else
	{
		if ( m_cloudData.getCpdDone() )
		{
			pca->calculatePCA(m_cloudData.getBigW(), n_latent, pcaMethod);
			m_cloudData.setPca(pca);
			m_cloudData.setPcaDone(true);

			MatrixXd zero = MatrixXd::Zero(1, pca->getNumLatent());
			MatrixXd deformed  = getModelFromLatent(zero);

			m_cloudData.setTransformedTesting(deformed);
			m_viewer.updateCloudsTesting();

			return true;
		}
		else
		{
			ROS_INFO( "Please load or calculate CPD first" );
			return false;
		}
	}
}


// Make a big W matrix with the W from each instance in a single row
void ShapeRegistration::buildBigW()
{
	if ((m_using_meshes && !m_meshData.getCpdDone() ) || ( !m_using_meshes && !m_cloudData.getCpdDone() ) )
	{
		ROS_INFO_STREAM( "Please calculate CPD" );
		return;
	}
	else
	{
		std::vector<MatrixXd> allW;
		int notCanonicalNumber = 0;
		int numberInstances;
		int canonicalNumber;

		// Meshes
		if (m_using_meshes)
		{
			allW = m_meshData.getAllW();

			if (notCanonicalNumber == m_meshData.getCanonicalIdx())
			{
				notCanonicalNumber = 1;
			}

			numberInstances = m_meshData.getNumberInstances();
			canonicalNumber = m_meshData.getCanonicalIdx();
		}
		// Clouds
		else
		{
			allW = m_cloudData.getAllW();

			if (notCanonicalNumber == m_cloudData.getCanonicalIdx())
			{
				notCanonicalNumber = 1;
			}

			numberInstances = m_cloudData.getNumberInstances();
			canonicalNumber = m_cloudData.getCanonicalIdx();
		}

		// Builds one big matrix with one instance in each row
		MatrixXd bigW(numberInstances - 1, allW[notCanonicalNumber].cols() * allW[notCanonicalNumber].rows());
		int countModel = 0;

		for (int modelNumber = 0; modelNumber < numberInstances; modelNumber++)
		{
			if (modelNumber != canonicalNumber)
			{
				int index = 0;

				for (int i = 0; i < allW[modelNumber].rows(); i++)
				{
					for (int j = 0; j < allW[modelNumber].cols(); j++)
					{
						bigW(countModel, index) = allW[modelNumber](i, j);
						index++;
					}
				}

				countModel++;
			}
		}

		if (m_using_meshes)
		{
			m_meshData.setBigW( bigW );
		}
		else
		{
			m_cloudData.setBigW( bigW );
		}
	}
}


void ShapeRegistration::setSlider(int value)
{
	double percent = value / 100.0;

	m_viewer.setSliderValue(percent);

	updateViewer();
}


void ShapeRegistration::visualizePCA()
{
	if ((m_using_meshes && m_meshData.getPcaDone() == false) || (m_using_meshes == false && m_cloudData.getPcaDone() == false))
	{
		ROS_INFO_STREAM( "Please calculate the Shape Space (PCA) first" );
	}
	else
	{
		int notCanonicalNumber = 0;
		if (notCanonicalNumber == m_meshData.getCanonicalIdx())
		{
			notCanonicalNumber = 1;
		}

		pca::ptr calculatedPca;

		if (m_using_meshes)
			calculatedPca = m_meshData.getPca();
		else
			calculatedPca = m_cloudData.getPca();

		// Build a vector to choose components to take in account
		int latent = calculatedPca->getNumLatent();

		MatrixXd varChooser = MatrixXd::Zero(1, latent);
		varChooser(0, m_latentVariable1) = m_latentVisualization1;
		ROS_INFO_STREAM( "latent variable number " << m_latentVariable1 << " with value " <<  m_latentVisualization1 );

		if (m_latentVariable1 != m_latentVariable2)
		{
			varChooser(0, m_latentVariable2) = m_latentVisualization2;
			ROS_INFO_STREAM("latent variable number " << m_latentVariable2 << " with value " << m_latentVisualization2 );
		}

		// Transform this vector into the observed space
		MatrixXd newW = calculatedPca->transformXStar(varChooser);

		MatrixXd transformW = wFromClassW(0, newW);

		// Calculate the transformation with the new W matrix
		MatrixXd canonical;
		MatrixXd G;
		MatrixXd tmpTransformed;

		if (m_using_meshes)
		{
			canonical = m_meshData.m_canonicalMat;
			G = m_meshData.getG();
			tmpTransformed = canonical + G * transformW ;

			m_meshData.m_transformedPCAMesh = matrixToMesh(tmpTransformed, m_meshData.getTransformedAt(notCanonicalNumber));
			m_viewer.updateMeshesTesting();
		}
		else
		{
			canonical = m_cloudData.m_canonicalMat;
			G = m_cloudData.getG();
			tmpTransformed = canonical + G * transformW ;

			m_cloudData.setTransformedTesting( matrixToCloud(tmpTransformed , 0 , 0, 255) );
			m_viewer.updateCloudsTesting();
		}
	}
}


// Transform one column from bigW to the corresponding W
MatrixXd ShapeRegistration::wFromClassW(int index, MatrixXd bigW)
{
	MatrixXd output(bigW.cols() / 3, 3);
	int count = 0;

	for (int i = 0; i < output.rows(); i++)
	{
		for (int j = 0; j < output.cols(); j++)
		{
			output(i, j) = bigW(index, count);
			count++;
		}
	}

	return output;
}


void ShapeRegistration::saveCPD()
{
	if ((m_using_meshes && m_meshData.getCpdDone() == false) || (m_using_meshes == false && m_cloudData.getCpdDone() == false) )
	{
		ROS_INFO_STREAM( "please calculate a cpd" );
	}
	else
	{
		buildBigW();
		MatrixXd G;
		MatrixXd W;
		ofstream saving;

		if (m_using_meshes)
		{
			G = m_meshData.getG();
			W = m_meshData.getBigW();

			// Saving canonical number
			saving.open (m_category_filepath + "/meshCpdSaving.txt");
			saving << m_meshData.getCanonicalIdx();
			saving.close();
			saving.open (m_category_filepath  + "/meshCpdSaving.txt", std::ofstream::out | std::ofstream::app);
		}
		else
		{
			G = m_cloudData.getG();
			W = m_cloudData.getBigW();

			// Saving canonical number
			saving.open (m_category_filepath  + "/cloudCpdSaving.txt");
			saving << m_cloudData.getCanonicalIdx();
			saving.close();
			saving.open (m_category_filepath  + "/cloudCpdSaving.txt", std::ofstream::out | std::ofstream::app);
		}

		saving << "\n------\n";

		// Save G
		for (int i = 0; i < G.rows() ; i++)
		{
			for (int j = 0; j < G.cols(); j++)
			{
				saving << G(i, j) << " ";
			}

			saving << "\n" ;
		}

		saving << "------\n";

		// Save W
		for (int i = 0; i < W.rows() ; i++)
		{
			for (int j = 0; j < W.cols(); j++)
			{
				saving << W(i, j) << " ";
			}

			saving << "\n" ;
		}

		saving.close();

		ROS_INFO_STREAM("data saved in " << m_category_filepath );
	}
}


void ShapeRegistration::loadCPD()
{
	if ((m_using_meshes && m_meshData.getCpdDone() ) || (m_using_meshes == false && m_cloudData.getCpdDone() ) )
	{
		ROS_INFO_STREAM( "there is already a CPD" );   //TODO handle when there is already a CPD
	}
	else
	{
		int readCanonicalNumber;
		MatrixXd readBigW;
		MatrixXd readG;

		int indexW = 0;
		int indexG = 0;
		int status = 0 ;
		std::string line;
		ifstream saving;

		if (m_using_meshes)
		{
			saving.open(m_category_filepath + "/meshCpdSaving.txt");
		}
		else
		{
			saving.open(m_category_filepath + "/cloudCpdSaving.txt");
		}

		if (saving.is_open())
		{
			while ( getline (saving, line) )
			{
				std::stringstream stream(line);

				if (status == 0)
				{
					if (line.find("------") != std::string::npos)
					{
						status = 1;
					}
					else
					{
						stream >> readCanonicalNumber;
					}
				}
				// After the first ----- G is saved
				else if (status == 1)
				{
					if (line.find("------") != std::string::npos)
					{
						status = 2;
					}
					else
					{
						int colCount = 0;
						double n;

						while (stream >> n)
						{
							readG.conservativeResize(indexG + 1, std::max((int)readG.cols(), colCount + 1));
							readG(indexG, colCount) = n;
							colCount++;
						}

						indexG++;
					}
				}
				// After the second ----- W is saved
				else if (status == 2)
				{
					int colCount = 0;
					double n;

					while (stream >> n)
					{
						readBigW.conservativeResize(indexW + 1, std::max((int)readBigW.cols(), colCount + 1));
						readBigW(indexW, colCount) = n;
						colCount++;
					}

					indexW++;
				}
			}

			saving.close();

			// Save the loaded Informations to the category
			if (m_using_meshes)
			{
				m_meshData.setCanonicalIdx(readCanonicalNumber);
				m_meshData.setG(readG);
				m_meshData.setBigW(readBigW);
			}
			else
			{
				m_cloudData.setCanonicalIdx(readCanonicalNumber);
				m_cloudData.setG(readG);
				m_cloudData.setBigW(readBigW);
			}

			buildFromCPD();
		}
		else
		{
			ROS_INFO_STREAM( "Unable to open file" );
		}
	}
}


void ShapeRegistration::buildFromCPD()
{
	if (m_using_meshes)
	{
		buildMeshesFromCPD();
	}
	else
	{
		buildCloudsFromCPD();
	}
}


// Build the transformations fom loaded CPD data
void ShapeRegistration::buildMeshesFromCPD()
{
	if (false) 
		ROS_INFO_STREAM( "Please load the meshes belonging to this CPD and click again" ); 
		// conditions when this will not work
	else
	{
		// Subsample the canonical Mesh in the same way it happened when the CPD was calculated
		vtkSmartPointer<vtkPolyData> subCan = vtkSmartPointer<vtkPolyData>::New();
		subCan->DeepCopy(m_meshData.getCanonical() );
		subCan = resolution(subCan, 1);

		while (subCan->GetNumberOfPoints() > m_meshData.getG().rows())
		{
			subCan = quadricClustering(subCan, 32);
		}

		m_meshData.m_canonicalMat = meshToMatrix(subCan);

		std::vector<MatrixXd> allW;

		// Run through all instances and calculate the transformations
		for (int i = 0; i < (int)m_meshData.getNumberInstances(); i++)
		{
			MatrixXd transformedMatrix = m_meshData.m_canonicalMat;

			if ( i != m_meshData.getCanonicalIdx())
			{
				MatrixXd Wi;

				if (i < m_meshData.getCanonicalIdx())
					Wi = wFromClassW(i , m_meshData.getBigW());

				if (i > m_meshData.getCanonicalIdx())
					Wi = wFromClassW(i - 1, m_meshData.getBigW());

				transformedMatrix = m_meshData.m_canonicalMat + m_meshData.getG() * Wi;
				m_meshData.setTransformedAt(i, matrixToMesh(transformedMatrix, subCan));
				allW.push_back(Wi);
			}
			else
			{
				m_meshData.setTransformedAt(i, matrixToMesh(m_meshData.m_canonicalMat, subCan));
				MatrixXd A(1, 1);
				A << 0;
				allW.push_back(A);
			}
		}

		m_meshData.setAllW(allW);

		// Save an instance for visualizing PCA after the CPD for visualization purposes
		m_meshData.m_transformedPCAMesh = matrixToMesh(m_meshData.m_canonicalMat, subCan, 0, 0 , 255);

		m_meshData.setCpdDone(true);
		m_viewer.updateTraMesh();

		ROS_INFO_STREAM( "CPD is loaded" );
	}
}


void ShapeRegistration::buildCloudsFromCPD()
{
	if (false)
		ROS_INFO_STREAM( "Please load the clouds belonging to this CPD and click again" ); 
		// conditions when this will not work
	else
	{
		PointCloudT::Ptr canonical;
		canonical.reset(new pcl::PointCloud<PointT> ());
		canonical = m_cloudData.getCanonical();

		m_cloudData.m_canonicalMat = cloudToMatrix(canonical);

		std::vector<MatrixXd> allW;

		// Run through all instances and calculate the transformations
		for (int i = 0; i < (int)m_cloudData.getNumberInstances(); i++)
		{
			MatrixXd transformedMatrix = m_cloudData.m_canonicalMat;

			if ( i != m_cloudData.getCanonicalIdx())
			{
				MatrixXd Wi;

				if (i < m_cloudData.getCanonicalIdx())
					Wi = wFromClassW(i , m_cloudData.getBigW());

				if (i > m_cloudData.getCanonicalIdx())
					Wi = wFromClassW(i - 1, m_cloudData.getBigW());

				transformedMatrix = m_cloudData.m_canonicalMat + m_cloudData.getG() * Wi;
				m_cloudData.setTransformedAt(i, matrixToCloud(transformedMatrix));
				allW.push_back(Wi);
			}
			else
			{
				m_cloudData.setTransformedAt(i, matrixToCloud(m_cloudData.m_canonicalMat));
				MatrixXd A(1, 1);
				A << 0;
				allW.push_back(A);
			}
		}

		m_cloudData.setAllW(allW);

		m_cloudData.setCpdDone(true);
		m_viewer.updateTraCloud();

		ROS_INFO_STREAM( "CPD is loaded" );
	}
}


MatrixXd ShapeRegistration::pointsInLatentSpace()
{
	pca::ptr calculatedPca;

	if (m_using_meshes)
		calculatedPca = m_meshData.getPca();
	else
		calculatedPca = m_cloudData.getPca();

	MatrixXd allLatent = calculatedPca->getX();
	MatrixXd latentPoints(2, allLatent.rows());

	for (int i = 0; i < latentPoints.cols(); i++)
	{
		latentPoints(0, i) = allLatent(i,  m_latentVariable1);

		if ( m_latentVariable1 == m_latentVariable2 )
			latentPoints(1, i) = 0;
		else
			latentPoints(1, i) = allLatent(i,  m_latentVariable2);
	}

	return latentPoints;
}


void ShapeRegistration::clear()
{
	MeshManager tmp;
	m_meshData = tmp;

	CloudManager tmpCloud ;
	m_cloudData = tmpCloud;
}


void ShapeRegistration::fitToObserved()
{
	if (!m_using_meshes)
	{
		if ( m_cloudData.getPcaDone() )
		{
			MatrixXd can_matrix = cloudToMatrix( m_cloudData.getCanonical() );

			m_solver_thread->updatePCA( m_cloudData.getPca() );
			m_solver_thread->updateCanonicalMatrix(can_matrix);

			MatrixXd obs_matrix = cloudToMatrix( m_cloudData.getObservedTesting() );
			MatrixXd latent = MatrixXd::Zero(1, m_cloudData.getPca()->getNumLatent() );
			Eigen::Affine3d local_rigid = Eigen::Affine3d::Identity();

			emit fit( obs_matrix, latent, local_rigid );
		}
		else
			ROS_WARN("The latent space (PCA) has to be calculated before.");
	}
}


void ShapeRegistration::cancelFitting()
{
	emit haltOpt();
}


MatrixXd ShapeRegistration::getModelFromLatent(const MatrixXd& latent, const Eigen::Affine3d& trans)
{
	MatrixXd newW = m_cloudData.getPca()->transformXStar(latent);
	MatrixXd transformW = wFromClassW(0, newW);
	MatrixXd tmpTransformed = m_cloudData.m_canonicalMat + m_cloudData.getG() * transformW ;

	MatrixXd aligned = tmpTransformed;

	for (int i = 0; i < tmpTransformed.rows(); i++)
		aligned.row(i) = (trans * Eigen::Vector3d( tmpTransformed.row(i).transpose()) ).transpose();

	return aligned;
}


MatrixXd ShapeRegistration::getModelFromLatent(const MatrixXd& latent, double *pose)
{
	// Retrieve the resulting pose
	Eigen::Affine3d local_rigid;
	Eigen::Affine3d t( Eigen::Translation3d(Eigen::Vector3d(pose[0], pose[1], pose[2])) );
	Eigen::Vector3d axis(pose[3], pose[4], pose[5]);
	axis.normalize();
	Eigen::Affine3d r ( Eigen::AngleAxisd( pose[6], axis) );
	local_rigid = t * r;

	return getModelFromLatent(latent, local_rigid);
}


void ShapeRegistration::fitted(const MatrixXd& latent, const Eigen::Affine3d& trans)
{
	MatrixXd aligned = getModelFromLatent(latent, trans);

	if (m_using_meshes)
	{
		m_cloudData.setTransformedTesting(matrixToCloud(aligned));
		m_viewer.updateCloudsTesting();
	}

	ROS_INFO_STREAM( "Inference Results\n Latent:" << latent << "\n Local Rigid Trans:\n" << trans.matrix() );
}


void ShapeRegistration::plotCallback()
{
}


void ShapeRegistration::modelCallback(double *latent, double *pose)
{
	int nLatent = m_cloudData.getPca()->getNumLatent();

	MatrixXd latent_vector = MatrixXd::Zero(1, nLatent);

	for (int i = 0; i < nLatent; i++)
		latent_vector(0, i) = latent[i];

	MatrixXd aligned = getModelFromLatent(latent_vector, pose);

	m_cloudData.setTransformedTesting(aligned);
	m_viewer.updateCloudsTesting();
}
