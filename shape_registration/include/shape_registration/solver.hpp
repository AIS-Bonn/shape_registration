// Categorical Shape Registration
// Author: Corbin Cogswell <corbincogswell@gmail.com>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#pragma once

#include <QObject>

#include <shape_registration/pca.hpp>

#include <ceres/ceres.h>

#include <Eigen/Geometry>


typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;


enum CostFunction
{
	POINT = 0,
	POINT_RIGID,
	MIN_DISTANCE,
	COST_FUNCTIONS_COUNT
};


class cost_function
{
public:
	cost_function() {};
	~cost_function() {};

	virtual bool operator()(double const* const* parameters, double* residuals) const = 0;
};


// Gradient descent solver for point-to-point matching
class cost_function_point_cpd : public cost_function
{
public:
	cost_function_point_cpd(const pca::ptr& PCA, const MatrixXd& canonical, const MatrixXd& observed, bool optSigma, double sigma);

	bool operator()(double const* const* parameters, double* residuals) const;

private:
	pca::ptr m_PCA;			// In the PCA object resides the learned latent space
	MatrixXd m_G;			// Gaussian kernel of the canonical model
	MatrixXd m_canonical;
	MatrixXd m_observed;

	bool m_optSigma;
	double m_sigma;

	static const double Wuniform;   // For accounting for outliers
	static const double beta;       // For calculating the G kernel matrix

	// Parameters
	const float m_costFunGain = 50000.0;
	const float m_regLatentGain = 10.0;
	const float m_regTranslationGain = 0.0;
	const float m_regRotationGain = 0.0;

	const bool m_rigidEnabled = true;
};


// Gradient descent solver for point-to-point matching for RIGID registration
class cost_function_point_rigid : public cost_function
{
public:
	cost_function_point_rigid(const MatrixXd& canonical, const MatrixXd& observed, double sigma);
	bool operator()(double const* const* parameters, double* residuals) const;

private:
	MatrixXd m_canonical;
	MatrixXd m_observed;

	double m_sigma;

	static const double Wuniform;

	// Config parameters
	const float m_costFunGain = 5000.0;
	const float m_regPoseGain = 0.0;
};


// Gradient descent solver for point-to-point minimum distance
class cost_function_min_distance : public cost_function
{
public:
	cost_function_min_distance(const pca::ptr& PCA, const MatrixXd& canonical, const MatrixXd& observed);
	bool operator()(double const* const* parameters, double* residuals) const;

private:
	pca::ptr m_PCA;
	MatrixXd m_G;
	MatrixXd m_canonical;
	MatrixXd m_observed;

	static const double beta;       // For calculating the G kernel matrix

	const bool m_rigidEnabled = true;
};


class fit_iteration_callback : public QObject, public ceres::IterationCallback
{
	Q_OBJECT

public:
	fit_iteration_callback();
	~fit_iteration_callback();

	ceres::CallbackReturnType operator()(const ceres::IterationSummary&);

	void setParameters(double* XStar, double* pose);

	void setHalt(bool halt);

signals:
	void plotCallback();
	void modelCallback(double* XStar, double* pose);

private:
	double* m_XStar;
	double* m_pose;
	bool m_halt;
};
