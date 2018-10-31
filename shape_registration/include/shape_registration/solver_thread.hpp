// Categorical Shape Registration
// Author: Corbin Cogswell <corbincogswell@gmail.com>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#pragma once

// QT
#ifndef Q_MOC_RUN
#include <QThread>
#include <QReadWriteLock>
#endif

#include <shape_registration/pca.hpp>
#include <shape_registration/solver.hpp>

// Eigen
#include <Eigen/Geometry>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;


class solver_thread : public QThread
{
	Q_OBJECT

public:
	solver_thread(QReadWriteLock* mutex);
	~solver_thread();

	void updateCanonicalMatrix(const MatrixXd &canonical_matrix);
	void updatePCA(const pca::ptr PCA);

	fit_iteration_callback* getCallback();

protected:
	void run();

private:
	void solve();

	QReadWriteLock* m_mutex;

	pca::ptr m_PCA;

	MatrixXd m_canonical_matrix;
	MatrixXd m_observed_matrix;
	MatrixXd m_XStar;

	Eigen::Affine3d m_local_rigid;

	fit_iteration_callback* m_callback;

	// Config parameters
	const float m_sigma = 0.05;
	const int m_max_iterations = 40;
	const int m_cost_function = 2;
	const bool m_opt_sigma = false;

signals:
	void fitted(const MatrixXd& XStar, const Eigen::Affine3d& trans);

public slots:
	void fit(MatrixXd observed_matrix, const MatrixXd& XStar, const Eigen::Affine3d& trans);
	void halt();
};
