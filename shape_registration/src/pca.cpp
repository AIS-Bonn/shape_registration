// Computation of PCA and PCA-EM using Eigen
// Author: Corbin Cogswell <corbincogswell@gmail.com>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#include <shape_registration/pca.hpp>

#include <ros/console.h>

#include <vector>
#include <Eigen/Eigenvalues>


pca::pca()
{
}


pca::pca(int n_latent)
{
	m_num_latent = n_latent;
}


pca::pca(const pca& PCA)
{
	m_rows = PCA.getNumRows();
	m_cols = PCA.getNumCols();
	m_num_latent = PCA.getNumLatent();

	m_W = PCA.m_W;

	m_X = PCA.m_X;
	m_Ymu = PCA.m_Ymu;
	m_Ystd = PCA.m_Ystd;

	m_pcaPercent = PCA.m_pcaPercent;

	m_method = PCA.m_method;
}


pca::~pca()
{
}


pca::ptr pca::New()
{
	pca::ptr PCA;
	PCA.reset(new pca());
	return PCA;
}


pca::ptr pca::clone()
{
	pca::ptr PCA;
	PCA.reset(new pca(*this));
	return PCA;
}


void pca::centerData(const Eigen::MatrixXd &Yin, Eigen::MatrixXd &Yout)
{
	Yout.resize(Yin.rows(), Yin.cols());
	m_Ymu = Yin.colwise().mean();

	Yout = Yin.rowwise() - m_Ymu.row(0);
}


void pca::scaleData(const Eigen::MatrixXd &Yin, Eigen::MatrixXd &Yout)
{
	Yout.resize(Yin.rows(), Yin.cols());

	m_Ystd = ((Yin.rowwise() - m_Ymu.row(0)).array().square().colwise().sum() / (Yin.rows() - 1)).sqrt();

	for (uint i = 0; i < m_Ystd.cols(); ++i)
		if (m_Ystd(0, i) == 0)
			m_Ystd(0, i) = 1e-30;

	Yout = (Yin.rowwise() - m_Ymu.row(0)).array().rowwise() / m_Ystd.row(0).array();

	ROS_DEBUG_STREAM("Standarized data: \n" << Yout );
}


void pca::calculatePCA(const Eigen::MatrixXd &Y, const double param, PCA_METHOD method)
{
	m_Yraw = Y;
	m_rows = m_Yraw.rows();
	m_cols = m_Yraw.cols();

	centerData(m_Yraw, m_Ycentered);         // This calculates m_Ymu

	if (method == PCA_COV)
	{
		m_method = method;
		calculatePCACov(m_Ycentered, param);
	}
	else if (method == PCA_EM)
	{
		m_method = method;

		if ( std::floor(param) > 0)
			m_num_latent = std::floor(param);

		if (m_num_latent > 0)
			calculatePCAEM(m_Ycentered, m_num_latent);
		else
			calculatePCAEM(m_Ycentered);
	}
	else
	{
		ROS_ERROR("Can't perform PCA. A unknown method to calculate PCA was given.");
		return;
	}
}


void pca::calculatePCACov(const Eigen::MatrixXd& Y, const double targetPCAPercent)
{
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>solver(Y.adjoint()*Y);

	// Eigenvectors and eigenvalues in descending order
	Eigen::MatrixXd eigenVectors = solver.eigenvectors().real().rowwise().reverse();
	Eigen::VectorXd eigenValues = solver.eigenvalues().real().reverse();

	ROS_DEBUG_STREAM( "EigenVectors: \n" << eigenVectors << "\n");
	ROS_DEBUG_STREAM( "EigenValues: \n" << eigenValues << "\n");

	// Calculate the percentage of variance from each component
	std::vector<double> eigenPercents = std::vector<double>(m_cols);
	std::vector<double> eigenSums = std::vector<double>(m_cols);

	double sum = 0.0;

	for (int i = 0; i < m_cols; i++)
		sum += eigenValues(i);

	double eSum = 0.0;
	int pca_params = 0;
	bool set = false;

	for (int i = 0; i < m_cols; i++)
	{
		eigenPercents[i] = eigenValues(i) / sum;
		eSum += eigenPercents[i];
		eigenSums[i] = eSum;

		if (eSum >= targetPCAPercent && !set)
		{
			pca_params = i + 1;
			m_pcaPercent = eigenSums[i];
			set = true;
		}
	}

	// Store the transforming matrix
	m_num_latent = pca_params;
	m_W = Eigen::MatrixXd(eigenVectors.leftCols(m_num_latent)); // q x n_rows
	m_X = transformYStar(m_Yraw);

	ROS_INFO_STREAM ("PCA-Cov, for capturing " << m_pcaPercent * 100 << "% of variability, " << m_num_latent << " latent varaibles are required");
}


bool pca::calculatePCAEM(const Eigen::MatrixXd &Yin, const int nLatent)
{
	m_num_latent = nLatent;

	Eigen::MatrixXd lastW;
	m_W = Eigen::MatrixXd::Random(m_num_latent, m_cols);

	for (int i = 0; i < m_iterations; i++)
	{
		lastW = m_W;
		// E-step
		m_X = Yin * m_W.transpose() * (m_W * m_W.transpose()).inverse();
		// M-step
		m_W = (m_X.transpose() * m_X).inverse() * m_X.transpose() * Yin;

		if ( (m_W - lastW).array().abs().maxCoeff() < m_empca_tol)
		{
			if (i > m_min_iterations)
			{
				ROS_INFO_STREAM("PCA-EM finished in " << i << " iterations. Error: " << (m_W - lastW).array().abs().maxCoeff() );
				m_X = transformYStar(m_Yraw);
				return true;
			}
		}
	}

	ROS_INFO_STREAM("PCA-EM reaches the maximum number of iterations:  " << m_iterations << ". Error: " <<  (m_W - lastW).array().abs().maxCoeff() );

	// Get latent variables of canonical model
	m_X = transformYStar(m_Yraw);

	return false;
}


int pca::getNumRows() const
{
	return m_rows;
}

int pca::getNumCols() const
{
	return m_cols;
}

int pca::getNumLatent() const
{
	return m_num_latent;
}

Eigen::MatrixXd pca::getX() const
{
	return m_X;
}

Eigen::MatrixXd pca::getYmu() const
{
	return m_Ymu;
}

Eigen::MatrixXd pca::getYstd() const
{
	return m_Ystd;
}

Eigen::MatrixXd pca::getW() const
{
	return m_W;
}


// Converts from latent to original observed (not centered)
Eigen::MatrixXd pca::transformXStar(const Eigen::MatrixXd& XStar)
{
	// ROS_DEBUG_STREAM("Xstar: " << XStar << "\n" );
	// ROS_DEBUG_STREAM( "W: " << m_W << "\n" );

	Eigen::MatrixXd YStar;

	if (m_method == PCA_EM)
		YStar = XStar * m_W;
	else if (m_method == PCA_COV)
		YStar = XStar * m_W.transpose();
	else
		ROS_ERROR_STREAM("Unable to transform the latent data to the observed space. The PCA method is not defined or unknown. Method: " << m_method);

	for (unsigned int i = 0; i < XStar.rows(); i++)
	{
		if (m_bStandarized)
			// Elementwise multiplication sum by the mean
			YStar.row(i) = YStar.row(i).array() * m_Ystd.row(0).array() + m_Ymu.row(0).array();
		else
			YStar.row(i) = YStar.row(i) + m_Ymu.row(0);
	}

	return YStar;
}


// Converts from observed (not centered) to latent
Eigen::MatrixXd pca::transformYStar(const Eigen::MatrixXd& YStar)
{
	Eigen::MatrixXd Yc(YStar.rows(), m_cols);
	Eigen::MatrixXd Ystd = m_Ystd;

	for (unsigned int i = 0; i < YStar.rows(); i++)
	{
		if (m_bStandarized)
			Yc.row(i) = (YStar.row(i) - m_Ymu.row(0)).array() / m_Ystd.row(0).array();
		else
			Yc.row(i) = YStar.row(i) - m_Ymu.row(0);
	}

	// ROS_DEBUG_STREAM("Yc: " << Yc << "\n");
	// ROS_DEBUG_STREAM("W: " << m_W << "\n");

	Eigen::MatrixXd XStar;

	if (m_method == PCA_EM)
		XStar = Yc * m_W.transpose() * (m_W * m_W.transpose()).inverse() ;
	else if (m_method == PCA_COV)
		XStar = Yc * m_W;
	else
		ROS_ERROR_STREAM( "Unable to transform the data to the laten space. The PCA method is not defined or unknown. Method: " << m_method );

	return XStar;
}
