#pragma once
#include <string>
#include <Eigen/Sparse>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkTriangle.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkPLYReader.h>
#include <igl/readMESH.h>

template <typename DerivedV, typename DerivedF>
vtkSmartPointer<vtkPolyData> MakePolyData(	Eigen::PlainObjectBase<DerivedV>& V, Eigen::PlainObjectBase<DerivedF>& F){
	// vtkNew<vtkDoubleArray> pointsArray;
	// pointsArray->SetArray(V.data(), V.size(), 0);	
	// pointsArray->SetNumberOfComponents(3);	
	vtkNew<vtkPoints> points;
	// points->SetData(pointsArray); // <-- directly assign pointer
	std::cout << "#############" << std::endl;
	for(int vid=0 ; vid < V.rows() ; vid++){		
		points->InsertNextPoint(V(vid, 0), V(vid, 1), V(vid, 2));
	}

	vtkNew<vtkCellArray> triangles;
	for(int fid=0 ; fid < F.rows() ; fid++){
		vtkNew<vtkTriangle> triangle;
		triangle->GetPointIds()->SetId(0, F(fid, 0) );		
		triangle->GetPointIds()->SetId(1, F(fid, 1) );		
		triangle->GetPointIds()->SetId(2, F(fid, 2) );

		triangles->InsertNextCell(triangle);
	}

	vtkNew<vtkPolyData> polydata;
	polydata->SetPoints(points);
	polydata->SetPolys(triangles);

	return polydata;
}

vtkSmartPointer<vtkPolyData> ReadPolyData(std::string filename){
	
	vtkSmartPointer<vtkPolyData> results;
	
	std::string ext = filename.substr(filename.find_last_of(".")+1);
	if(ext == "off"){
		//read igl off file read test
		// Eigen::Matrix<double, -1, -1,  Eigen::RowMajor> V;
		Eigen::MatrixXd V;
		Eigen::MatrixXi F;	
		Eigen::MatrixXi T;	
		igl::read_triangle_mesh(filename, V,F);
		
		results = MakePolyData(V, F);

	}else if(ext == "obj"){
		vtkNew<vtkOBJReader> reader;
		reader->SetFileName(filename.c_str());
		reader->Update();
		results = reader->GetOutput();
	}else if(ext == "ply"){
		vtkNew<vtkPLYReader> reader;
		reader->SetFileName(filename.c_str());
		reader->Update();
		results = reader->GetOutput();
	}

	return results;
}


vtkSmartPointer<vtkActor> MakeActor(vtkSmartPointer<vtkPolyData> polydata){

	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(polydata);

	vtkNew<vtkActor> actor;
	actor->SetMapper(mapper);


	return actor;
}