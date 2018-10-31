// Categorical Non-rigid registration
// Author: Florian Huber <s6flhube@uni-bonn.de>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#pragma once

#include <cpd/nonrigid.hpp>


namespace categorical_registration
{
	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;

	void initCPD();
	cpdG::NonrigidResult doCPD(MatrixXd fixed , MatrixXd moving, double beta = 1.0, double lambda = 1.0);
}
