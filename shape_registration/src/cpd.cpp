// Categorical shape Registration
// Author: Florian Huber <s6flhube@uni-bonn.de>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#include <shape_registration/cpd.hpp>
#include <ros/console.h>

using namespace categorical_registration;

cpdG::NonrigidResult categorical_registration::doCPD(MatrixXd fixed, MatrixXd moving, double beta, double lambda)
{
	// Make use of the Gadomski CPD
	cpdG::Nonrigid nonR;
	nonR.beta(beta);
	nonR.lambda(lambda);
	//nonR.correspondence(true);
	nonR.max_iterations(250);
	nonR.linked(true);
	nonR.normalize(false);

	ROS_INFO_STREAM( "CPD starts Calculating. Calculate with: " << fixed.rows() << " observed points, and " << moving.rows() << " canonical points" );

	std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
	cpdG::NonrigidResult ergG = nonR.run(fixed, moving);
	std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();

	MatrixXd erg = ergG.points;

	ROS_INFO_STREAM( "CPD is done in: " << std::chrono::duration_cast<std::chrono::microseconds> (t6 - t5).count() * 1.e-6  << " seconds. With " << ergG.iterations << " iterations" );
	return ergG;
}
