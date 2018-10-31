// Categorical Shape Registration
// Author: Corbin Cogswell <corbincogswell@gmail.com>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#include <shape_registration/solver.hpp>
#include <shape_registration/cpd.hpp>
#include <ros/console.h>


/// cost_function_point_no_normals
const double cost_function_point_cpd::Wuniform = 0.1;
const double cost_function_point_cpd::beta = 1.0;

cost_function_point_cpd::cost_function_point_cpd(const pca::ptr& PCA,
        const MatrixXd& canonical,
        const MatrixXd& observed,
        bool optSigma,
        double sigma):
	m_PCA(PCA),
	m_canonical(canonical),
	m_observed(observed),
	m_optSigma(optSigma),
	m_sigma(sigma)
{
	m_G = cpdG::affinity(m_canonical, m_canonical, beta);		// G Matrix calculated only when object is initialized
}


bool cost_function_point_cpd::operator()(double const* const* parameters, double* residuals) const
{
	int q = m_PCA->getNumLatent();

	// Latent variables
	MatrixXd XStar(1, q);

	for (int i = 0; i < q; i++)
	{
		XStar(0, i) = parameters[0][i];
	}

	MatrixXd W;
	W = m_PCA->transformXStar(XStar);

	int n = m_G.rows();
	int index = 0;
	MatrixXd WShaped(n, 3);

	for (int r = 0; r < n; r++)
	{
		for (int c = 0; c < 3; c++)
		{
			WShaped(r, c) = W(0, index++);
		}
	}

	MatrixXd Z = m_G * WShaped;
	MatrixXd deformed = m_canonical + Z;

	// Local Rigid Transformation
	Eigen::Affine3d t( Eigen::Translation3d(Eigen::Vector3d(parameters[1][0], parameters[1][1], parameters[1][2])) );

	// Normalized rotation axis
	Eigen::Vector3d axis(parameters[1][3], parameters[1][4], parameters[1][5]);
	axis.normalize();

	Eigen::Affine3d r( Eigen::AngleAxisd( parameters[1][6], axis) );

	Eigen::Affine3d local_rigid = t * r;

	MatrixXd aligned = deformed;

	if ( m_rigidEnabled )
	{
		for (int i = 0; i < deformed.rows(); i++)
			aligned.row(i) = local_rigid * Eigen::Vector3d( deformed.row(i).transpose() );
	}


	// Give a penalty for high values in the local transformation
	double translation_penalty = 0.0;
	double rotation_penalty = 0.0;

	if ( m_rigidEnabled )
	{
		translation_penalty = std::pow((0 - parameters[1][0]), 2) +
		                      std::pow((0 - parameters[1][1]), 2) +
		                      std::pow((0 - parameters[1][2]), 2);

		rotation_penalty = std::pow((0 - parameters[1][3]), 2) +
		                   std::pow((1 - parameters[1][4]), 2) +
		                   std::pow((0 - parameters[1][5]), 2) +
		                   std::pow((0 - parameters[1][6]), 2);
	}

	// Penalty for higher values of the latent variables
	double xPenalty = 0.0;

	for (int i = 0; i < q; i++)
	{
		xPenalty += std::pow(XStar(0, i), 2);
	}

	// Create the importance to weight the distances
	int M = aligned.rows();
	int N = m_observed.rows();
	const double D = 3.0;

	// Sigma
	double sigma;

	if (m_optSigma)
		sigma = parameters[2][0];
	else
		sigma = m_sigma;

	double sigma2 = sigma * sigma;

	MatrixXd P = cpdG::affinity(aligned, m_observed, sigma);
	MatrixXd Pdiv = P.colwise().sum().array() + (std::pow(2 * M_PI * sigma2, D / 2.0) * Wuniform / (1 - Wuniform) * M / N);

	for (int m = 0; m < M; m++)
	{
		for (int n = 0; n < N; n++)
			P(m, n) = P(m, n) / Pdiv(0, n);
	}

	for (int i = 0; i < N; i++)
	{
		double dist = 0.0;

		for (int j = 0; j < M; j++)
		{
			dist += P(j, i) * std::pow( (m_observed.row(i) - aligned.row(j)).norm() , 2);
		}

		residuals[i] = m_costFunGain * dist + m_regLatentGain * xPenalty + m_regTranslationGain * translation_penalty + m_regRotationGain * rotation_penalty;
	}

	return true;
}


/// ******************   For RIGID REGISTRATION ***************************
const double cost_function_point_rigid::Wuniform = 0.1;

cost_function_point_rigid::cost_function_point_rigid(const MatrixXd& canonical,
        const MatrixXd& observed,
        double sigma):
	m_canonical(canonical),
	m_observed(observed),
	m_sigma(sigma)
{
}

bool cost_function_point_rigid::operator()(double const* const* parameters, double* residuals) const
{
	// Local Rigid Transformation
	Eigen::Affine3d t( Eigen::Translation3d(Eigen::Vector3d(parameters[1][0], parameters[1][1], parameters[1][2])) );

	// Normalized rotation axis
	Eigen::Vector3d axis(parameters[1][3], parameters[1][4], parameters[1][5]);
	axis.normalize();

	Eigen::Affine3d r( Eigen::AngleAxisd( parameters[1][6], axis) );

	Eigen::Affine3d local_rigid = t * r;

	MatrixXd aligned = m_canonical;

	for (int i = 0; i < m_canonical.rows(); i++)
		aligned.row(i) = local_rigid * Eigen::Vector3d( m_canonical.row(i).transpose() );

	int M = aligned.rows();
	int N = m_observed.rows();
	const double D = 3.0;

	// Give a penalty for high alignment parameters
	double pPenalty = 0.0;

	pPenalty = std::pow((0 - parameters[1][0]), 2) +
	           std::pow((0 - parameters[1][1]), 2) +
	           std::pow((0 - parameters[1][2]), 2) +
	           std::pow((0 - parameters[1][3]), 2) +
	           std::pow((1 - parameters[1][4]), 2) +
	           std::pow((0 - parameters[1][5]), 2) +
	           std::pow((0 - parameters[1][6]), 2);

	pPenalty /= N;

	// Sigma
	double sigma = m_sigma;
	double sigma2 = sigma * sigma;


	// Create the importance to weight the distances
	MatrixXd P = cpdG::affinity(aligned, m_observed, sigma);
	MatrixXd Pdiv = P.colwise().sum().array() + (std::pow(2 * M_PI * sigma2, D / 2.0) * Wuniform / (1 - Wuniform) * M / N);

	for (int m = 0; m < M; m++)
	{
		for (int n = 0; n < N; n++)
			P(m, n) = P(m, n) / Pdiv(0, n);
	}

	for (int i = 0; i < N; i++)
	{
		double dist = 0.0;

		for (int j = 0; j < M; j++)
		{
			dist += P(j, i) * std::pow( (m_observed.row(i) - aligned.row(j)).norm() , 2);
		}

		residuals[i] = m_costFunGain * dist + m_regPoseGain * pPenalty;
	}

	return true;
}


/// ******************  Minimum distance cost_function ***************************
const double cost_function_min_distance::beta = 1.0;

cost_function_min_distance::cost_function_min_distance(const pca::ptr& PCA, const MatrixXd& canonical, const MatrixXd& observed):
	m_PCA(PCA),
	m_canonical(canonical),
	m_observed(observed)
{
	m_G = cpdG::affinity(m_canonical, m_canonical, beta);		// G Matrix calculated only when object is initialized
}


bool cost_function_min_distance::operator()(double const* const* parameters, double* residuals) const
{
	// Get latent variables
	int q = m_PCA->getNumLatent();

	MatrixXd XStar(1, q);

	for (int i = 0; i < q; i++)
		XStar(0, i) = parameters[0][i];

	// Local Rigid Transformation
	Eigen::Affine3d t( Eigen::Translation3d(Eigen::Vector3d(parameters[1][0], parameters[1][1], parameters[1][2])) );

	// Normalized rotation axis
	Eigen::Vector3d axis(parameters[1][3], parameters[1][4], parameters[1][5]);
	axis.normalize();

	Eigen::Affine3d r( Eigen::AngleAxisd( parameters[1][6], axis) );

	Eigen::Affine3d local_rigid = t * r;

	MatrixXd W;
	W = m_PCA->transformXStar(XStar);

	int index = 0;
	MatrixXd WShaped(m_G.rows(), 3);

	for (unsigned int r = 0; r < m_G.rows(); r++)
	{
		for (int c = 0; c < 3; c++)
			WShaped(r, c) = W(0, index++);
	}

	// Warp the model
	MatrixXd Z = m_G * WShaped;
	MatrixXd deformed = m_canonical + Z;

	// Apply rotation and translation to the deformed model
	MatrixXd aligned = deformed;

	if ( m_rigidEnabled )
	{
		for (int i = 0; i < deformed.rows(); i++)
			aligned.row(i) = (local_rigid * Eigen::Vector3d( deformed.row(i).transpose()) ).transpose();
	}

	// Distance to the closest point
	int M = aligned.rows();
	int N = m_observed.rows();

	for (int i = 0; i < N; i++)
	{
		double dist = 0.0;
		double min = INT_MAX;

		for (int j = 0; j < M; j++)
		{
			dist = (m_observed.row(i) - aligned.row(j)).squaredNorm();

			if (dist < min)
			{
				min = dist;
			}
		}

		residuals[i] = min;
	}

	return true;
}


/// fit_iteration_callback
fit_iteration_callback::fit_iteration_callback()
{
	m_halt = false;
}

fit_iteration_callback::~fit_iteration_callback()
{
}

void fit_iteration_callback::setParameters(double* XStar, double* pose)
{
	m_XStar = XStar;
	m_pose = pose;
}

void fit_iteration_callback::setHalt(bool halt)
{
	m_halt = halt;
}

ceres::CallbackReturnType fit_iteration_callback::operator()(const ceres::IterationSummary&)
{
	emit plotCallback();
	emit modelCallback(m_XStar, m_pose);

	ceres::CallbackReturnType r;

	if (m_halt)
	{
		r = ceres::SOLVER_ABORT;
	}
	else
	{
		r = ceres::SOLVER_CONTINUE;
	}

	return r;
}

