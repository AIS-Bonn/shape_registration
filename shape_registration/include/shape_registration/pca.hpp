// Computation of PCA and PCA-EM using Eigen
// Author: Corbin Cogswell <corbincogswell@gmail.com>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#pragma once

#include <Eigen/Geometry>

#include <memory>       // shared pointers

class pca
{

public:
	enum PCA_METHOD
	{
		PCA_COV = 1,
		PCA_EM = 2,
		PCA_COUNT = PCA_EM
	};

	pca();
	pca(int n_latent);
	pca(const pca& PCA);
	~pca();

	typedef std::shared_ptr<pca> ptr;
	static pca::ptr New();
	pca::ptr clone();

	// Calculates W, Ymu, and X from Y
	void calculatePCA(const Eigen::MatrixXd& Y, const double param, PCA_METHOD method);
	void calculatePCACov(const Eigen::MatrixXd& Y, const double targetPCAPercent);

	// Calculates PCA using Expectation Maximization
	bool calculatePCAEM(const Eigen::MatrixXd &Yin, const int nLatent = 10);

	int getNumRows() const;               // Number of rows of the input matrix
	int getNumCols() const;               // Number of columns of the input matrix
	int getNumLatent() const;             // Number of latent variables

	Eigen::MatrixXd getX() const;
	Eigen::MatrixXd getYmu() const;
	Eigen::MatrixXd getYstd() const;
	Eigen::MatrixXd getW() const;       // q-Reduced transform

	Eigen::MatrixXd transformXStar(const Eigen::MatrixXd& XStar); // Converts from latent to observed (denormalized data)
	Eigen::MatrixXd transformYStar(const Eigen::MatrixXd& YStar); // Converts from observed (denormalized data) to latent

private:
	void centerData(const Eigen::MatrixXd &Yin, Eigen::MatrixXd &Yout);
	void scaleData(const Eigen::MatrixXd &Yin, Eigen::MatrixXd &Yout);

	int m_rows;
	int m_cols;
	int m_num_latent = 0;

	PCA_METHOD m_method;

	bool m_bStandarized = false;

	// PCA training results
	Eigen::MatrixXd m_W;            // transforming matrix
	Eigen::MatrixXd m_Yraw;         // Raw data without normalization
	Eigen::MatrixXd m_Ycentered;  // Normalized input data
	Eigen::MatrixXd m_Ymu;          // Mean of the raw unnormalize data
	Eigen::MatrixXd m_Ystd;         // Std of the raw unnormalize data
	Eigen::MatrixXd m_X;            // Latent matrix

	double m_pcaPercent;

	// EM consts
	const int m_iterations = 100;       // Max number of iterations
	const int m_min_iterations = 10;    // Min number of iterations
	const double m_empca_tol = 1e-9;
};
