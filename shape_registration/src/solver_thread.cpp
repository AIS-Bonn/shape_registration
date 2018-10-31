// Categorical Shape Registration
// Author: Corbin Cogswell <corbincogswell@gmail.com>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#include <shape_registration/solver_thread.hpp>

#include <ros/console.h>


solver_thread::solver_thread(QReadWriteLock* mutex)
	: QThread()
{
	qRegisterMetaType<MatrixXd>("MatrixXd");
	qRegisterMetaType<Eigen::Affine3d>("Eigen::Affine3d");

	m_mutex = mutex;

	m_callback = new fit_iteration_callback();
	google::InitGoogleLogging("Shape registration: non-linear optimizer");
}


solver_thread::~solver_thread()
{
	wait();
	delete m_callback;
}


void solver_thread::run()
{
	solve();
	m_mutex->lockForRead();
	m_mutex->unlock();
}


void solver_thread::updateCanonicalMatrix(const MatrixXd &canonical_matrix)
{
	m_mutex->lockForWrite();
	m_canonical_matrix = canonical_matrix;
	m_mutex->unlock();
}


void solver_thread::updatePCA(pca::ptr PCA)
{
	m_mutex->lockForWrite();
	m_PCA = PCA;
	m_mutex->unlock();
}


fit_iteration_callback* solver_thread::getCallback()
{
	return m_callback;
}


void solver_thread::fit(MatrixXd observed_matrix, const MatrixXd& XStar, const Eigen::Affine3d& trans)
{
	ROS_DEBUG("Solver got signal");

	if (!isRunning())
	{
		m_mutex->lockForRead();
		m_observed_matrix = observed_matrix;
		m_XStar = XStar;
		m_local_rigid = trans;
		m_mutex->unlock();
		m_callback->setHalt(false);
		start(LowPriority);
	}
}


void solver_thread::halt()
{
	if (isRunning())
	{
		m_callback->setHalt(true);
		wait();
	}
}


void solver_thread::solve()
{
	// Make copies so we do not need to lock up when making assignments
	m_mutex->lockForRead();

	pca::ptr PCA = m_PCA->clone();
	int q = PCA->getNumLatent();
	MatrixXd XStar = m_XStar;
	Eigen::Affine3d local_rigid = m_local_rigid;
	double sigma[] = {m_sigma};   // Initial value

	m_mutex->unlock();

	int n = m_observed_matrix.rows();

	double transformationArray[7];
	transformationArray[0] = local_rigid.translation().x();
	transformationArray[1] = local_rigid.translation().y();
	transformationArray[2] = local_rigid.translation().z();

	Eigen::AngleAxisd rot( local_rigid.rotation() );
	transformationArray[3] = rot.axis().x();
	transformationArray[4] = rot.axis().y();
	transformationArray[5] = rot.axis().z();
	transformationArray[6] = rot.angle();

	// Instantiate the ceres cost function
	ceres::DynamicNumericDiffCostFunction<cost_function, ceres::CENTRAL>* costFunction;

	switch ( m_cost_function )
	{
		case POINT:
			costFunction = new ceres::DynamicNumericDiffCostFunction<cost_function, ceres::CENTRAL>(
			    new cost_function_point_cpd(PCA, m_canonical_matrix, m_observed_matrix, m_opt_sigma, m_sigma ));
			costFunction->SetNumResiduals(n);
			break;

		case POINT_RIGID:
			costFunction = new ceres::DynamicNumericDiffCostFunction<cost_function, ceres::CENTRAL>(
			    new cost_function_point_rigid(m_canonical_matrix, m_observed_matrix, m_sigma ));
			costFunction->SetNumResiduals(n);
			break;

		// By default the distance to the closest point is used as the cost function
		default:
			costFunction = new ceres::DynamicNumericDiffCostFunction<cost_function, ceres::CENTRAL>(
			    new cost_function_min_distance(PCA, m_canonical_matrix, m_observed_matrix) );
			costFunction->SetNumResiduals(n);
			break;
	}

	// Instantiate the solver
	ceres::Problem problem;
	ceres::Solver::Options options;
	ceres::Solver::Summary summary;

	options.minimizer_progress_to_stdout = true;
	options.update_state_every_iteration = true;
	options.max_num_iterations = m_max_iterations;

	// Set parameters for updating the GUI
	m_callback->setParameters(XStar.data(), transformationArray);
	options.callbacks.push_back(m_callback);

	// Configure solver for the latent space registration
	costFunction->AddParameterBlock(q);    // Latent Variables
	costFunction->AddParameterBlock(7);    // Pose

	if (m_opt_sigma)
	{
		costFunction->AddParameterBlock(1);    // Sigma
		problem.AddResidualBlock(costFunction, NULL, XStar.data(), transformationArray, sigma );
	}
	else
		problem.AddResidualBlock(costFunction, NULL, XStar.data(), transformationArray );

	// Run the solver
	ROS_INFO_STREAM( "Xstar before: " << XStar << "\n") ;

	ceres::Solve(options, &problem, &summary);
	ROS_INFO_STREAM(summary.BriefReport());

	// Retrieve the resulting pose
	Eigen::Affine3d t( Eigen::Translation3d(Eigen::Vector3d(transformationArray[0], transformationArray[1], transformationArray[2])) );
	Eigen::Vector3d axis(transformationArray[3], transformationArray[4], transformationArray[5]);
	axis.normalize();
	Eigen::Affine3d r ( Eigen::AngleAxisd( transformationArray[6], axis) );
	local_rigid = t * r;

	if (summary.termination_type == ceres::CONVERGENCE)
	{
		ROS_INFO("Fitting successful");
	}
	else
	{
		ROS_WARN("Maximum number of iterations reached OR fitting failed");
	}

	m_mutex->lockForWrite();
	m_XStar = XStar;
	m_local_rigid = local_rigid;
	m_mutex->unlock();

	emit fitted(m_XStar, m_local_rigid);
}
