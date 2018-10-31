// Categorical Shape Registration
// Author: Florian Huber <s6flhube@uni-bonn.de>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#include <shape_registration/gui.hpp>
#include "ui_surface_gui.h"

#include <pcl/filters/filter.h>
#include <pcl/conversions.h>


using namespace categorical_registration;

ShapeGui::ShapeGui (QWidget *parent) :
	QMainWindow (parent),
	ui (new Ui::ShapeGui)
{
	ui->setupUi (this);
	this->setWindowTitle ("Shape Registration GUI");

	m_shape_reg.reset(new(ShapeRegistration));

	// Both Frames
	connect (ui->canonicalCheckBox, SIGNAL (stateChanged(int)), this, SLOT (showCanonical(int)));
	connect (ui->observedCheckBox, SIGNAL (stateChanged(int)), this, SLOT (showObserved(int)));
	connect (ui->deformedCheckBox, SIGNAL (stateChanged(int)), this, SLOT (showDeformed(int)));
	connect (ui->visSlider, SIGNAL (valueChanged(int)), this, SLOT (visSliderChanged(int)));
	
	connect (ui->frameWidget, SIGNAL (tabBarClicked(int)), this , SLOT (frameChangePushed(int)));
	
	// Training Frame
	connect (ui->cloudButton, SIGNAL (clicked()), this, SLOT (cloudRadioPushed()));
	connect (ui->meshButton, SIGNAL (clicked()), this, SLOT (meshRadioPushed()));
	connect (ui->listWidget,  SIGNAL (itemClicked(QListWidgetItem*)), this, SLOT (setObservedPushed(QListWidgetItem*)));

	connect (ui->newCategoryButton, SIGNAL (clicked()), this, SLOT (newCategoryPushed()));
	connect (ui->loadCategoryButton, SIGNAL (clicked()), this, SLOT (loadCategoryPushed()));
	connect (ui->saveCategoryButton, SIGNAL (clicked()), this, SLOT (saveCategoryPushed()));
	connect (ui->loadMeshesButton,  SIGNAL (clicked ()), this, SLOT (loadInstancesPushed()));
	connect (ui->canonicalButton, SIGNAL (clicked()), this, SLOT (setCanonicalPushed()));
	connect (ui->deleteButton, SIGNAL (clicked()), this, SLOT (deleteEntryPushed()));
	connect (ui->buildButton, SIGNAL (clicked()), this, SLOT (buildSetPushed()));

	// Testing Frame
	connect (ui->pcaButton, SIGNAL (clicked()), this, SLOT (pcaPushed()));
	connect (ui->numLatentBox, SIGNAL (valueChanged(int)), this, SLOT (numberLatentChanged(int)));
	connect (ui->latentBox1, SIGNAL (valueChanged(int)), this, SLOT (latentVarChanged1(int)));
	connect (ui->latentBox2, SIGNAL (valueChanged(int)), this, SLOT (latentVarChanged2(int)));

	connect (ui->pointCloudFileOpenButton, SIGNAL(clicked()), this, SLOT(getObservedPointCloudFromFile()));
	connect(ui->pointCloudTopicBox, SIGNAL(currentIndexChanged(int)), this, SLOT(getPointCloudFromTopic()));

	connect (ui->fitButton, SIGNAL(clicked()), this, SLOT(fitToObserved()));
	connect (ui->cancelFittingButton, SIGNAL(clicked()), this, SLOT(cancelFitting()));

	//Set scale of the QwtPlot
	ui->qwtPlot->setAxisScale (0, -10, 10, 5);
	ui->qwtPlot->setAxisScale (2, -10, 10, 5);

	// This makes the selected list item green
	QPalette p = ui->listWidget->palette();
	p.setColor(QPalette::Highlight, Qt::green);
	ui->listWidget->setPalette(p);

	m_clicked = new QwtPlotMarker();

	// Make the QwtPlot ready for mouse events
	QwtPlotPicker* plotPicker = new QwtPlotPicker(ui->qwtPlot->xBottom, ui->qwtPlot->yLeft, QwtPicker::CrossRubberBand, QwtPicker::AlwaysOn, ui->qwtPlot->canvas());
	QwtPickerMachine* pickerMachine = new QwtPickerClickPointMachine();
	plotPicker->setStateMachine(pickerMachine);
	connect(plotPicker, SIGNAL(selected(const QPointF &)), this, SLOT(plotClicked(const QPointF &)));

	// Begin the main loop
	timer = new QTimer(this);
	timer->setInterval(10); // in ms
	QWidget::connect(timer, SIGNAL(timeout()), this, SLOT(loop()));
	timer->start();

	// Get the list of point cloud topics
	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);
	ui->pointCloudTopicBox->addItem(QObject::tr(" "));

	for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++)
	{
		const ros::master::TopicInfo& info = *it;

		ROS_INFO_STREAM("Topic name: " << info.name );

		if (info.datatype == "sensor_msgs/PointCloud2")
		{
			ui->pointCloudTopicBox->addItem(QString::fromStdString(info.name));
		}
	}

	m_FilePathsMeshes.clear();
	m_FilePathsClouds.clear();
}


ShapeGui::~ShapeGui ()
{
	delete ui;
	delete timer;
}


// The main loop function
void ShapeGui::loop()
{
	ros::spinOnce();
	m_shape_reg->viewer()->spinOnce();
}


void ShapeGui::resizeEvent(QResizeEvent *event)
{
	if (m_timerId)
	{
		killTimer(m_timerId);
		m_timerId = 0;
	}

	m_timerId = startTimer(10);  /*delay beetween ends of resize and your action*/
}


void ShapeGui::timerEvent(QTimerEvent *event)
{
	resizeGradientPlot();

	killTimer(event->timerId());
	m_timerId = 0;
}


void ShapeGui::resizeGradientPlot()
{
	const QRect cr = ui->qwtPlot->canvas()->contentsRect();

	QPointF center;
	center.setX( (cr.topLeft().x() + cr.topRight().x()) / 2.);
	center.setY( (cr.topLeft().y() + cr.bottomLeft().y()) / 2.);
	double radius = std::sqrt( std::pow((cr.width() / 2.), 2) + pow(cr.height() / 2., 2 ) );

	QRadialGradient gradient(center, radius);
	gradient.setColorAt(0.0, QColor::fromRgbF(0.8, 0.8, 0.8, 1) );
	gradient.setColorAt(1.0, QColor::fromRgbF(0, 0, 0, 1) );

	QPalette pal = ui->qwtPlot->canvas()->palette();
	pal.setBrush(QPalette::Window, QBrush(gradient));
	ui->qwtPlot->canvas()->setPalette(pal);

	ui->qwtPlot->replot();
	ui->qwtPlot->updateLayout();
}


void ShapeGui::showCanonical(int state)
{
	if (state == Qt::Checked)
		m_shape_reg->viewer()->showCanonical(true);
	else
		m_shape_reg->viewer()->showCanonical(false);

	m_shape_reg->updateViewer();
}


void ShapeGui::showObserved(int state)
{
	if (state == Qt::Checked)
		m_shape_reg->viewer()->showObserved(true);
	else
		m_shape_reg->viewer()->showObserved(false);

	m_shape_reg->updateViewer();
}


void ShapeGui::showDeformed(int state)
{
	if (state == Qt::Checked)
		m_shape_reg->viewer()->showDeformed(true);
	else
		m_shape_reg->viewer()->showDeformed(false);

	m_shape_reg->updateViewer();
}


void ShapeGui::visSliderChanged(int value)
{
	m_shape_reg->setSlider(value);
}


void ShapeGui::newCategoryPushed()
{
	QString text = QInputDialog::getText(this, "New category", "enter name: ", QLineEdit::Normal, "");

	// Display the current category
	std::string name = text.toStdString();
	std::string category_info = "current category : " + name;
	ui->categoryLabel->setText(QString::fromUtf8(category_info.c_str()));

	name = ros::package::getPath("shape_registration") + "/data/" + name ;
	m_shape_reg->setCategoryFilepath (name);
	mkdir(name.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	std::ofstream stream1;
	stream1.open(name + "/paths.txt") ;
	stream1.close();
}


void ShapeGui::loadCategoryPushed()
{
	std::string startFolder = ros::package::getPath("shape_registration") + "/data/";
	QFileDialog dialog;
	dialog.setDirectory( QString::fromStdString(startFolder));
	QString dir  = dialog.getExistingDirectory(this, tr("Open Directory"),
											   QString::fromStdString(startFolder),
											   QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
	std::string folder = dir.toStdString();

	std::ifstream file;

	if (folder.size() > 0)
	{
		std::string category_name;
		category_name = folder.substr(folder.find_last_of("/\\") + 1);

		if (category_name.size() < 2)
		{
			category_name = folder;
			category_name.erase(category_name.find_last_of("/\\"));
			category_name = category_name.substr(category_name.find_last_of("/\\") + 1);
		}

		category_name = "current category : " + category_name;
		ui->categoryLabel->setText(QString::fromUtf8(category_name.c_str()));
		m_shape_reg->setCategoryFilepath (folder);
		file.open(folder + "/paths.txt");
	}

	if (file.is_open())
	{
		// Delete all old informations
		ui->listWidget->clear();
		m_shape_reg->clear();
		m_FilePathsMeshes.clear();
		m_FilePathsClouds.clear();
		int status = 0;
		std::string line;

		while ( getline (file, line) )
		{
			std::stringstream stream(line);

			if (status == 0)
			{
				if (line.find("------") != std::string::npos)
				{
					status = 1;
				}
				else
				{
					ROS_INFO_STREAM( "The file paths.txt is wrong forametted" );
				}
			}
			// After the first ----- meshes are saved
			else if (status == 1)
			{
				if (line.find("------") != std::string::npos)
				{
					status = 2;
				}
				else
				{
					m_FilePathsMeshes.push_back(line);

					if (ui->meshButton->isChecked() == true)
					{
						boost::filesystem::path p(line);
						ui->listWidget->addItem(QString::fromStdString(p.stem().string()));
					}
				}
			}
			// After the second ----- clouds are saved
			else if (status == 2)
			{
				m_FilePathsClouds.push_back(line);

				if (ui->cloudButton->isChecked() == true)
				{
					boost::filesystem::path p(line);
					ui->listWidget->addItem(QString::fromStdString(p.stem().string()));
				}
			}
		}

		file.close();
		setModels();

		ifstream cpd_available;

		if (ui->meshButton->isChecked() == true)
		{
			cpd_available.open(m_shape_reg->getCategoryFilepath() + "/meshCpdSaving.txt");
		}
		else
		{
			cpd_available.open(m_shape_reg->getCategoryFilepath() + "/cloudCpdSaving.txt");
		}

		if (cpd_available.is_open())
		{
			loadCpd();
		}

		m_shape_reg->updateViewer();
	}
	else
	{
		ROS_INFO_STREAM( "Unable to open file" );
	}
}


void ShapeGui::saveCategoryPushed()
{
	if (m_shape_reg->getCategoryFilepath().length() < 1)
	{
		ROS_INFO_STREAM ( "First build a new category or load an existing one" );
	}
	else
	{
		std::ofstream file;
		file.open (m_shape_reg->getCategoryFilepath() + "/paths.txt");

		if (file.is_open() == false)
			ROS_INFO_STREAM("Could not save in : " << m_shape_reg->getCategoryFilepath() );
		else
		{
			file << "------meshes \n" ;
			file.close();
			file.open(m_shape_reg->getCategoryFilepath() + "/paths.txt",  std::ofstream::out | std::ofstream::app);

			for (unsigned int i = 0; i < m_FilePathsMeshes.size(); i++)
			{
				file << m_FilePathsMeshes[i] << "\n" ;
			}

			file << "------clouds \n" ;

			for (unsigned int i = 0; i < m_FilePathsClouds.size(); i++)
			{
				file << m_FilePathsClouds[i] << "\n" ;
			}

			file.close();
			ROS_INFO_STREAM("data saved saved in : " << m_shape_reg->getCategoryFilepath());
		}

		m_shape_reg->saveCPD();
	}
}


void ShapeGui::setObservedPushed(QListWidgetItem* item)
{
	if (ui->listWidget->currentItem()->background() == Qt::NoBrush)
		ui->listWidget->currentItem()->setBackground(Qt::green);

	int current;

	if (ui->meshButton->isChecked() )
		current = m_shape_reg->getObservedMeshIdx();
	else
		current = m_shape_reg->getObservedPCDIdx();

	if (ui->listWidget->count() > current)
	{
		if (ui->listWidget->item(current)->background() == Qt::green)
		{
			ui->listWidget->item(current)->setBackground(Qt::NoBrush);
		}
	}

	int idx = ui->listWidget->currentRow();

	if (ui->meshButton->isChecked() )
	{
		m_shape_reg->setObservedMeshIdx(idx);
	}
	else
	{
		m_shape_reg->setObservedPCDIdx(idx);
	}
}


void ShapeGui::loadInstancesPushed()
{
	if (m_shape_reg->getCategoryFilepath().length() < 1)
	{
		ROS_INFO_STREAM ( "First build a new category or load an existing one" );
	}
	else
	{
		QStringList fileNames;
		std::string startFolder = ros::package::getPath("shape_registration");

		if ( ui->meshButton->isChecked() )
		{
			// Opens QFileDialog to search for ply files
			QStringList tmp = QFileDialog::getOpenFileNames(this, tr("Open File"),  QString::fromStdString(startFolder), tr("PLY Files (*.ply)"));
			fileNames = tmp;
		}
		else
		{
			// Opens QFileDialog to search for pcd files
			QStringList tmp = QFileDialog::getOpenFileNames(this, tr("Open File"),  QString::fromStdString(startFolder), tr("PCD Files (*.pcd)"));
			fileNames = tmp;
		}

		for (int i = 0; i < fileNames.size(); i++)
		{
			QString fn = fileNames.at(i);
			std::string text = fn.toUtf8().constData();

			if (ui->meshButton->isChecked() == true)
				m_FilePathsMeshes.push_back(text);
			else
				m_FilePathsClouds.push_back(text);

			boost::filesystem::path p(text);
			ui->listWidget->addItem(QString::fromStdString(p.stem().string()));
		}

		if ( fileNames.size() > 0 )
			setModels();
	}
}


void ShapeGui::deleteEntryPushed()
{
	if (m_shape_reg->getCategoryFilepath().length() < 1)
	{
		ROS_INFO_STREAM ( "First build a new category or load an existing one" );
	}
	else if (ui->listWidget->count() < 1)
	{
		ROS_INFO_STREAM ( "Load some instances first" );
	}
	else
	{
		int idx = ui->listWidget->currentRow();

		if (ui->meshButton->isChecked() == true)
		{
			m_shape_reg->deleteMesh(idx);
			m_FilePathsMeshes.erase (m_FilePathsMeshes.begin() + ui->listWidget->currentRow() );
		}
		else
		{
			m_shape_reg->deletePCD(idx);
			m_FilePathsClouds.erase (m_FilePathsClouds.begin() + ui->listWidget->currentRow() );
		}

		qDeleteAll(ui->listWidget->selectedItems());
	}
}


void ShapeGui::setModels()
{
	// If instances already exist
	if (ui->meshButton->isChecked() && m_shape_reg->getMeshCount() > 0)
	{
		for (uint i = m_shape_reg->getMeshCount(); i < m_FilePathsMeshes.size(); i++)
		{
			m_shape_reg->addTrainingInstance(m_FilePathsMeshes[i]);
		}
	}
	else if (ui->meshButton->isChecked() == false && m_shape_reg->getCloudCount() > 0)
	{
		for (uint i = m_shape_reg->getCloudCount(); i < m_FilePathsClouds.size(); i++)
		{
			m_shape_reg->addTrainingInstance(m_FilePathsClouds[i]);
		}
	}
	// If this is the first time instances are added
	else
	{
		for (uint i = 0; i < m_FilePathsMeshes.size(); i++)
		{
			m_shape_reg->addTrainingInstance(m_FilePathsMeshes[i]);
		}

		for (uint i = 0; i < m_FilePathsClouds.size(); i++)
		{
			m_shape_reg->addTrainingInstance(m_FilePathsClouds[i]);
		}
	}

	if (ui->listWidget->count() > 1)
	{
		if (ui->meshButton->isChecked() == true)
		{
			ui->listWidget->item(m_shape_reg->getCanonicalMeshIdx())->setBackground(Qt::red);
			ui->listWidget->item(m_shape_reg->getObservedMeshIdx())->setBackground(Qt::green);
		}
		else
		{
			ui->listWidget->item(m_shape_reg->getCanonicalPCDIdx())->setBackground(Qt::red);
			ui->listWidget->item(m_shape_reg->getObservedPCDIdx())->setBackground(Qt::green);
		}
	}
}


void ShapeGui::setCanonicalPushed()
{
	if (m_shape_reg->getCategoryFilepath().length() < 1)
	{
		ROS_INFO_STREAM ( "First build a new category or load an existing one" );
	}
	else if (ui->listWidget->count() < 1)
	{
		ROS_INFO_STREAM ( "Load some instances first" );
	}
	else
	{
		ui->listWidget->currentItem()->setBackground(Qt::red);
		int current;

		if (ui->meshButton->isChecked() == true)
			current = m_shape_reg->getCanonicalMeshIdx();
		else
			current = m_shape_reg->getCanonicalPCDIdx();

		ui->listWidget->item(current)->setBackground(Qt::NoBrush);
		int idx = ui->listWidget->currentRow();

		if (ui->meshButton->isChecked() )
		{
			m_shape_reg->setCanonicalMeshIdx(idx);
		}
		else
		{
			m_shape_reg->setCanonicalPCDIdx(idx);
		}
	}
}


void ShapeGui::buildSetPushed()
{
	m_shape_reg->calculateDeformationFields();
}


void ShapeGui::frameChangePushed(int frame)
{
	if (frame == 0)
	{
		// Training
		m_shape_reg->trainingView(true);

		if (ui->meshButton->isChecked() )
		{
			m_shape_reg->viewer()->removeMeshesTesting();
			m_shape_reg->viewer()->updateMeshes();
		}
		else
		{
			m_shape_reg->viewer()->removeCloudsTesting();
			m_shape_reg->viewer()->updateClouds();
		}
	}
	else
	{
		// Testing
		m_shape_reg->trainingView(false);
		
		// Calculate directly PCA with the default number of latent variables
		pcaPushed();
		
		// Trigger a callback to update the latent plot
		m_timerId = startTimer(10);

		if (ui->meshButton->isChecked() )
		{
			m_shape_reg->viewer()->removeMeshes();
			m_shape_reg->viewer()->updateMeshesTesting();
		}
		else
		{
			m_shape_reg->viewer()->removeClouds();
			m_shape_reg->viewer()->updateCloudsTesting();
		}
	}
}


void ShapeGui::numberLatentChanged(int value)
{
	m_nLatent = value;
}


void ShapeGui::pcaPushed()
{
	m_nLatent = ui->numLatentBox->value();

	if ( m_nLatent > 0 )
	{
		if ( m_shape_reg->doPCA(m_nLatent) )
		{
			ui->latentBox1->setMaximum(m_nLatent - 1);
			ui->latentBox2->setMaximum(m_nLatent - 1);

			m_shape_reg->visualizePCA();
			updateLatentPlot();
		}
	}
	else
		ROS_WARN("The number of latent variables must be bigger than zero");
}


void ShapeGui::cloudRadioPushed()
{
	m_shape_reg->setUsingMeshes(false);

	ui->listWidget->clear();

	for (unsigned int i = 0; i < m_FilePathsClouds.size(); i++)
	{
		boost::filesystem::path p(m_FilePathsClouds[i]);
		ui->listWidget->addItem(QString::fromStdString(p.stem().string()));
	}

	if (ui->listWidget->count() > 1)
	{
		ui->listWidget->item(m_shape_reg->getCanonicalPCDIdx())->setBackground(Qt::red);
		ui->listWidget->item(m_shape_reg->getObservedPCDIdx())->setBackground(Qt::green);
	}

	// Load CPD when it is possible
	if (m_FilePathsClouds.size() > 2)
	{
		ifstream cpd_available;
		cpd_available.open(m_shape_reg->getCategoryFilepath() + "/cloudCpdSaving.txt");

		if (cpd_available.is_open())
		{
			loadCpd();
		}
	}
}


void ShapeGui::meshRadioPushed()
{
	m_shape_reg->setUsingMeshes(true);

	ui->listWidget->clear();

	for (unsigned int i = 0; i < m_FilePathsMeshes.size(); i++)
	{
		boost::filesystem::path p(m_FilePathsMeshes[i]);
		ui->listWidget->addItem(QString::fromStdString(p.stem().string()));
	}

	if (ui->listWidget->count() > 1)
	{
		ui->listWidget->item(m_shape_reg->getCanonicalMeshIdx())->setBackground(Qt::red);
		ui->listWidget->item(m_shape_reg->getObservedMeshIdx())->setBackground(Qt::green);
	}

	// Load CPD when it is possible
	if (m_FilePathsMeshes.size() > 2)
	{
		ifstream cpd_available;
		cpd_available.open(m_shape_reg->getCategoryFilepath() + "/meshCpdSaving.txt");

		if (cpd_available.is_open())
		{
			loadCpd();
		}
	}
}


void ShapeGui::latentVarChanged1(int value)
{
	m_shape_reg->setLatentVariable1 (value);
	m_shape_reg->visualizePCA();
	updateLatentPlot();
}


void ShapeGui::latentVarChanged2(int value)
{
	m_shape_reg->setLatentVariable2 (value);
	m_shape_reg->visualizePCA();
	updateLatentPlot();
}


void ShapeGui::plotClicked(const QPointF & point)
{
	double latent1 = (double) point.x();
	double latent2 = (double) point.y();

	// Add a marker to the clicked point
	QwtSymbol* sym = new QwtSymbol( QwtSymbol::Cross, QBrush(Qt::red), QPen(Qt::red, 3), QSize(8, 8) );
	m_clicked->setSymbol( sym );

	if (ui->latentBox1->value() == ui->latentBox2->value())
	{
		m_clicked->setValue(QPointF(latent1, 0));
	}
	else
	{
		m_clicked->setValue( point );
	}

	m_clicked->attach( ui->qwtPlot );
	ui->qwtPlot->replot();

	m_shape_reg->setLatentValue1 (latent1);
	m_shape_reg->setLatentValue2 (latent2);

	m_shape_reg->visualizePCA();
}


void ShapeGui::updateLatentPlot()
{
	MatrixXd mat = m_shape_reg->pointsInLatentSpace();
	std::vector<double> points(mat.data(), mat.data() + mat.rows() * mat.cols());

	for (unsigned int i = 0; i < points.size(); i++)
	{
		m_latents.size();

		if (m_latents.size() <= i / 2)
		{
			QwtPlotMarker* marker = new QwtPlotMarker();
			QwtSymbol* sym = new QwtSymbol( QwtSymbol::XCross, QBrush(Qt::blue), QPen(Qt::blue, 1), QSize(5, 5) );
			marker->setSymbol( sym );
			marker->setValue( QPointF(points[i], points[i + 1]));
			marker->attach( ui->qwtPlot );
			m_latents.push_back(marker);
			i++;
		}
		else
		{
			m_latents[i / 2]->setValue( QPointF(points[i], points[i + 1]));
			m_latents[i / 2]->attach( ui->qwtPlot );
			i++;
		}
	}

	ui->qwtPlot->replot();
}


void ShapeGui::loadCpd()
{
	m_shape_reg->loadCPD();
	int canonicalIdx = -1;

	if (m_shape_reg->usingMeshes() && m_shape_reg->cpdDoneOnMeshes() )
	{
		canonicalIdx = m_shape_reg->getCanonicalMeshIdx();
	}
	else if (!m_shape_reg->usingMeshes() && m_shape_reg->cpdDoneOnClouds() )
	{
		canonicalIdx = m_shape_reg->getCanonicalPCDIdx();
	}

	if (canonicalIdx > -1)
	{
		int current;

		if (ui->meshButton->isChecked() )
			current = m_shape_reg->getCanonicalMeshIdx();
		else
			current = m_shape_reg->getCanonicalPCDIdx();

		ui->listWidget->item(current)->setBackground(Qt::NoBrush);
		ui->listWidget->item(canonicalIdx)->setBackground(Qt::red);
	}
}


void ShapeGui::getObservedPointCloudFromFile()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open PointCloud File"), "/path/to/file/", tr("PCD Files (*.pcd)"));
	std::string text = fileName.toUtf8().constData();

	if (fileName.length() > 0)
	{
		m_shape_reg->addTestingObserved(text);

		boost::filesystem::path p(text);
		ui->pointCloudFileBox->setText(QString::fromStdString(p.stem().string() ));
	}
}


// For subscribing to a point cloud topic
void ShapeGui::getPointCloudFromTopic()
{
	QApplication::setOverrideCursor(Qt::WaitCursor);

	if (ui->pointCloudTopicBox->currentIndex() > 0)
	{
		ui->pointCloudFileBox->clear();
		std::string topic = ui->pointCloudTopicBox->currentText().toStdString();
		ros::NodeHandle nh;
		m_sub_cloud = nh.subscribe(topic, 1000, &ShapeGui::pointCloudCallback, this);
	}
	else
	{
		m_sub_cloud.shutdown();
	}

	QApplication::restoreOverrideCursor();
}


void ShapeGui::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	double timestamp = ros::Time::now().toSec();

	if (timestamp > (m_lastTimeStamp + 0.01))
	{
		m_lastTimeStamp = timestamp;

		pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
		fromROSMsg(*msg, pointCloud);

		std::vector<int> indices;
		pcl::removeNaNFromPointCloud<pcl::PointXYZRGB>(pointCloud, pointCloud, indices);

		int size = pointCloud.width * pointCloud.height;
		PointCloudT cloud(size, 1);

		for (int i = 0; i < size; i++)
		{
			cloud.points[i].x = pointCloud.points[i].x;
			cloud.points[i].y = pointCloud.points[i].y;
			cloud.points[i].z = pointCloud.points[i].z;

			if ( pcl::getFieldIndex(*msg, "rgb") > 0 )
			{
				cloud.points[i].r = pointCloud.points[i].r;
				cloud.points[i].g = pointCloud.points[i].g;
				cloud.points[i].b = pointCloud.points[i].b;
			}
			else
			{
				cloud.points[i].r = 0;
				cloud.points[i].g = 255;
				cloud.points[i].b = 0;
			}
		}

		PointCloudT::Ptr tmp_cloud( new PointCloudT(cloud) );
		m_shape_reg->setTestingObserved(tmp_cloud);
	}
}


void ShapeGui::fitToObserved()
{
	m_shape_reg->fitToObserved();
}


void ShapeGui::cancelFitting()
{
	m_shape_reg->cancelFitting();
}
