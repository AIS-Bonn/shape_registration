// Categorical Shape Registration
// Author: Florian Huber <s6flhube@uni-bonn.de>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>#pragma once

#pragma once

// Own package
#include <shape_registration/shape_registration.hpp>

// C++
#include <iostream>
#include <stdio.h>
#include <dirent.h>
#include <boost/filesystem.hpp>
#include <sys/stat.h>
#include <fstream>
#include <sstream>

// Qt
#include <QApplication>
#include <QMainWindow>
#include <QString>
#include <QListWidgetItem>
#include <QFileDialog>
#include <QInputDialog>
#include <QTimer>

// Qwt
#include <qwt_plot.h>
#include <qwt_symbol.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_picker.h>
#include <qwt_picker_machine.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>


namespace Ui
{
class ShapeGui;
}

namespace categorical_registration
{

class ShapeGui : public QMainWindow
{
	Q_OBJECT

public:
	explicit ShapeGui (QWidget *parent = 0);
	~ShapeGui ();

private Q_SLOTS:
	void frameChangePushed(int);

	void cloudRadioPushed();
	void meshRadioPushed();

	void newCategoryPushed();
	void loadCategoryPushed();
	void saveCategoryPushed();
	void loadInstancesPushed();
	void setCanonicalPushed();
	void deleteEntryPushed();
	void buildSetPushed();

	void setObservedPushed(QListWidgetItem*);

	void fitToObserved();
	void cancelFitting();
	void getObservedPointCloudFromFile();
	void getPointCloudFromTopic();

	void pcaPushed();
	void numberLatentChanged(int value);
	void latentVarChanged1(int value);
	void latentVarChanged2(int value);

	void plotClicked(const QPointF &);

	void showCanonical(int state);
	void showObserved(int state);
	void showDeformed(int state);
	void visSliderChanged(int value);

	void loop();

protected:
	void resizeEvent(QResizeEvent *event) override;
	void timerEvent(QTimerEvent *event);

private:
	void loadCpd();

	void resizeGradientPlot();
	void updateLatentPlot();

	void setModels();

	void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

	std::shared_ptr<ShapeRegistration> m_shape_reg;

	std::vector<std::string> m_FilePathsClouds;
	std::vector<std::string> m_FilePathsMeshes;

	int m_nLatent;

	Ui::ShapeGui *ui;
	QTimer* timer; // For main loop events

	// Last clicked point for the PCA visualization
	QwtPlotMarker* m_clicked;

	// Latent coordinates
	std::vector<QwtPlotMarker*> m_latents;

	int m_timerId = 0; // For resize events

	ros::Subscriber m_sub_cloud;	// Subscriber of point cloud topic

	double m_lastTimeStamp;
};

} // NS
