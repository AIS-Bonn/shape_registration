// Categorical Shape registration
// Author: Florian Huber <s6flhube@uni-bonn.de>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#include <shape_registration/gui.hpp>

using namespace categorical_registration;

int main (int argc, char *argv[])
{
	// Start the gui
	QApplication a (argc, argv);

	ros::init(argc, argv, "Surface_GUI");

	ShapeGui w;
	w.show ();

	return a.exec ();
}
