// Categorical Shape Registration
// Author: Florian Huber <s6flhube@uni-bonn.de>
//         Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#include <shape_registration/shape_registration.hpp>
#include <shape_registration/cpd.hpp>

#include <vtkExtractEdges.h>
#include <vtkPolyDataMapper.h>
#include <vtkLine.h>
#include <vtkTriangle.h>
#include <vtkProperty.h>
#include <vtkQuadricDecimation.h>
#include <vtkQuadricClustering.h>
#include <vtkTriangleFilter.h>

#include <vtkDataSetSurfaceFilter.h>
#include <vtkLoopSubdivisionFilter.h>
#include <vtkLinearSubdivisionFilter.h>

#include <ros/package.h>
#include <ros/console.h>

using namespace categorical_registration;

//Point Cloud related Methods------------------------------------------------------------------

PointCloudT::Ptr categorical_registration::colorCloud(PointCloudT::Ptr cloud, 
													  const unsigned int &r , 
													  const unsigned int &g, 
													  const unsigned int &b)
{
	for (unsigned int i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].r = r;
		cloud->points[i].g = g;
		cloud->points[i].b = b;
	}

	return cloud;
}


MatrixXd categorical_registration::cloudToMatrix(PointCloudT::Ptr cloud)
{
	MatrixXd Mat(cloud->points.size(), 3);

	for (unsigned int i = 0; i < cloud->points.size(); i++)
	{
		Mat(i, 0) = cloud->points[i].x;
		Mat(i, 1) = cloud->points[i].y;
		Mat(i, 2) = cloud->points[i].z;
	}

	return Mat;
}


PointCloudT::Ptr categorical_registration::matrixToCloud(MatrixXd mat, 
														 const unsigned int &r, 
														 const unsigned int &g, 
														 const unsigned int &b)
{
	PointCloudT::Ptr cloud;
	cloud.reset(new pcl::PointCloud<PointT> ());
	cloud->points.resize( mat.rows() );

	for (unsigned int i = 0; i < mat.rows(); i++)
	{
		cloud->points[i].x = mat(i, 0);
		cloud->points[i].y = mat(i, 1);
		cloud->points[i].z = mat(i, 2);
		cloud->points[i].r = r;
		cloud->points[i].g = g;
		cloud->points[i].b = b;
	}

	return cloud;
}


MatrixXd categorical_registration::meshToMatrix(vtkSmartPointer<vtkPolyData> mesh)
{
	MatrixXd Mat(mesh->GetNumberOfPoints(), 3);

	for (vtkIdType i = 0; i < mesh->GetNumberOfPoints(); i++)
	{
		double p[3];
		mesh->GetPoint(i, p);

		Mat(i, 0) = p[0];
		Mat(i, 1) = p[1];
		Mat(i, 2) = p[2];
	}

	return Mat;
}


PointCloudT::Ptr categorical_registration::transformCloud(PointCloudT::Ptr cloud_in, Eigen::Affine3d transform)
{
	PointCloudT::Ptr cloud_out;
	cloud_out.reset(new pcl::PointCloud<PointT> ());

	pcl::transformPointCloud(*cloud_in, *cloud_out, transform);

	return cloud_out;
}


//Mesh related Methods------------------------------------------------------------------------

vtkSmartPointer<vtkPolyData> categorical_registration::colorMesh(vtkSmartPointer<vtkPolyData> mesh, 
																 const unsigned int &r , 
																 const unsigned int &g, 
																 const unsigned int &b)
{
	// Setup colors
	unsigned char color[3] = {(unsigned char)r, (unsigned char)g, (unsigned char)b};

	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName ("Colors");

	for (int i = 0; i < mesh->GetPoints()->GetNumberOfPoints(); i++)
	{
		colors->InsertNextTupleValue(color);
	}

	mesh->GetPointData()->SetScalars(colors);
	return mesh;
}


// A method to build a "mesh" out of Points from a matrix. It is basically just a point cloud, because topology is missing
vtkSmartPointer<vtkPolyData> categorical_registration::matrixToMesh(MatrixXd mat, 
																	const unsigned int &r , 
																	const unsigned int &g, 
																	const unsigned int &b)
{
	vtkSmartPointer<vtkPolyData> mesh = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

	vtkIdType pid[1];

	for (unsigned int i = 0; i < mat.rows(); i++)
	{
		double px = mat.row(i)[0];
		double py = mat.row(i)[1];
		double pz = mat.row(i)[2];

		pid[0] = points->InsertNextPoint(px, py, pz);
		vertices->InsertNextCell(1, pid);
	}

	// Set the points and vertices we created as the geometry and topology of the polydata
	mesh->SetPoints(points);
	mesh->SetVerts(vertices);
	mesh = colorMesh(mesh, r, g, b);

	return mesh;
}


// A method to regain a mesh out of the (changed) point coordinates, when the topology of the mesh has not changed
vtkSmartPointer<vtkPolyData> categorical_registration::matrixToMesh(MatrixXd mat,
																	vtkSmartPointer<vtkPolyData> meshTop, 
																	const unsigned int &r, 
																	const unsigned int &g, 
																	const unsigned int &b)
{
	vtkSmartPointer<vtkPolyData> mesh = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

	vtkCellArray* polys = vtkCellArray::New();
	polys = meshTop->GetPolys();

	vtkIdType pid[1];

	for (unsigned int i = 0; i < mat.rows(); i++)
	{
		double px = mat.row(i)[0];
		double py = mat.row(i)[1];
		double pz = mat.row(i)[2];

		pid[0] = points->InsertNextPoint(px, py, pz);
		vertices->InsertNextCell(1, pid);

	}

	// Set the points and vertices we created as the geometry and topology of the polydata
	mesh->SetPoints(points);
	mesh->SetVerts(vertices);
	mesh->SetPolys(polys);
	mesh = colorMesh(mesh, r, g, b);

	return mesh;
}


// Subsample with quadric decimation algo
vtkSmartPointer<vtkPolyData> categorical_registration::quadricClustering(vtkSmartPointer<vtkPolyData> mesh, double factor)
{
	ROS_DEBUG_STREAM( "Before decimation \n ------------" );
	ROS_DEBUG_STREAM( "There are " << mesh->GetNumberOfPoints() << " points." );
	ROS_DEBUG_STREAM( "There are " << mesh->GetNumberOfPolys() << " polygons." );

	vtkSmartPointer<vtkQuadricClustering> decimate = vtkSmartPointer<vtkQuadricClustering>::New();

	decimate->SetNumberOfXDivisions(factor);
	decimate->SetNumberOfYDivisions(factor);
	decimate->SetNumberOfZDivisions(factor);
	decimate->SetInputData(mesh);
	decimate->Update();

	vtkSmartPointer<vtkPolyData> decimated = vtkSmartPointer<vtkPolyData>::New();
	decimated->ShallowCopy(decimate->GetOutput());

	ROS_DEBUG_STREAM( "After decimation \n ------------" );
	ROS_DEBUG_STREAM( "There are " << decimated->GetNumberOfPoints() << " points." );
	ROS_DEBUG_STREAM( "There are " << decimated->GetNumberOfPolys() << " polygons." );

	return decimated;
}


// Higher the resolution to fill get rid of big faces
vtkSmartPointer<vtkPolyData> categorical_registration::resolution(vtkSmartPointer<vtkPolyData> mesh, double factor)
{
	ROS_DEBUG_STREAM( "Before subdivision \n ------------" );
	ROS_DEBUG_STREAM( "There are " << mesh->GetNumberOfPoints() << " points." );
	ROS_DEBUG_STREAM( "There are " << mesh->GetNumberOfPolys() << " polygons." );

	// Triangulate at first
	vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
	triangleFilter->SetInputData(mesh);
	triangleFilter->Update();

	vtkSmartPointer<vtkPolyData> upSampled = vtkSmartPointer<vtkPolyData>::New();
	upSampled->DeepCopy(triangleFilter->GetOutput());

	vtkSmartPointer<vtkPolyDataAlgorithm> subdivisionFilter;
	subdivisionFilter = vtkSmartPointer<vtkLoopSubdivisionFilter>::New();
	dynamic_cast<vtkLoopSubdivisionFilter *> (subdivisionFilter.GetPointer())->SetNumberOfSubdivisions(factor);

	subdivisionFilter->SetInputData(upSampled);
	subdivisionFilter->Update();

	if (subdivisionFilter->GetOutput()->GetNumberOfPoints() == 0)
	{
		subdivisionFilter = vtkSmartPointer<vtkLinearSubdivisionFilter>::New();
		dynamic_cast<vtkLinearSubdivisionFilter *> (subdivisionFilter.GetPointer())->SetNumberOfSubdivisions(factor);
		subdivisionFilter->SetInputData(upSampled);
		subdivisionFilter->Update();
	}

	upSampled->ShallowCopy(subdivisionFilter->GetOutput());

	ROS_DEBUG_STREAM( "After subdivision \n ------------" );
	ROS_DEBUG_STREAM( "There are " << upSampled->GetNumberOfPoints() << " points." );
	ROS_DEBUG_STREAM( "There are " << upSampled->GetNumberOfPolys() << " polygons." );

	return upSampled;
}


// remove intrinsic geometry (to be finished)
vtkSmartPointer<vtkPolyData> categorical_registration::extractOutside(vtkSmartPointer<vtkPolyData> mesh)
{
	ROS_DEBUG_STREAM( "Before extraction \n ------------" );
	ROS_DEBUG_STREAM( "There are " << mesh->GetNumberOfPoints() << " points." );
	ROS_DEBUG_STREAM( "There are " << mesh->GetNumberOfPolys() << " polygons." );

	vtkSmartPointer<vtkDataSetSurfaceFilter> decimate = vtkSmartPointer<vtkDataSetSurfaceFilter>::New();

	decimate->SetInputData(mesh);
	decimate->Update();

	vtkSmartPointer<vtkPolyData> decimated = vtkSmartPointer<vtkPolyData>::New();
	decimated->ShallowCopy(decimate->GetOutput());

	ROS_DEBUG_STREAM( "After extraction \n ------------" );
	ROS_DEBUG_STREAM( "There are " << decimated->GetNumberOfPoints() << " points." );
	ROS_DEBUG_STREAM( "There are " << decimated->GetNumberOfPolys() << " polygons." );

	return decimated;
}
