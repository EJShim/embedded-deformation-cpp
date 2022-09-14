#include <iostream>
#include <fstream>
#include <string>
#include <vtkSmartPointer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <Eigen/Dense>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkPolyDataReader.h>
#include <vtkOpenGLSphereMapper.h>
#include <vtkProperty.h>
#include <vtkTriangle.h>
#include <vtkCellArray.h>
#include <random>
#include <vtkOBJReader.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>
#include <vtkDataArray.h>

#include <igl/read_triangle_mesh.h>


//TODO : optimize this
template <typename DerivedV, typename DerivedF>
vtkSmartPointer<vtkPolyData> MakePolyData(	Eigen::PlainObjectBase<DerivedV>& V, Eigen::PlainObjectBase<DerivedF>& F){
	vtkNew<vtkDoubleArray> pointsArray;
	pointsArray->SetArray(V.data(), V.size(), 0);	
	pointsArray->SetNumberOfComponents(3);	
	vtkNew<vtkPoints> points;
	points->SetData(pointsArray);			

	vtkNew<vtkCellArray> triangles;
	for(int fid=0 ; fid < F.rows() ; fid++){
		vtkNew<vtkTriangle> triangle;
		triangle->GetPointIds()->SetId(0, F.coeff(fid, 0) );		
		triangle->GetPointIds()->SetId(1, F.coeff(fid, 1) );		
		triangle->GetPointIds()->SetId(2, F.coeff(fid, 2) );

		triangles->InsertNextCell(triangle);
	}

	vtkNew<vtkPolyData> polydata;
	polydata->SetPoints(points);
	polydata->SetPolys(triangles);

	return polydata;
}


vtkSmartPointer<vtkActor> MakeActor(vtkSmartPointer<vtkPolyData> polydata){

	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(polydata);

	vtkNew<vtkActor> actor;
	actor->SetMapper(mapper);


	return actor;
}






int main(int argc, char *argv[]){
    
	// Initialize Renderer
    vtkNew<vtkRenderWindowInteractor> iren;
    iren->SetInteractorStyle(vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New());
    vtkNew<vtkRenderWindow> renWin;
    renWin->SetSize(1000, 1000);
    iren->SetRenderWindow(renWin);
    vtkNew<vtkRenderer> ren;
    renWin->AddRenderer(ren);
	ren->SetGradientBackground(true);
	ren->SetBackground(.2, .2, .2);
	ren->SetBackground2(.9, .9, .9);


	// Make Polydata and actor for rendering
	vtkNew<vtkOBJReader> reader;
	reader->SetFileName("../resources/sample_faust.obj");
	reader->Update();


	//read igl off file read test
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,  Eigen::RowMajor> V, U;
	Eigen::MatrixXi F;
	igl::read_triangle_mesh("../resources/decimated-knight.off", V, F);


	vtkSmartPointer<vtkPolyData> polydata = MakePolyData(V, F);		


	vtkSmartPointer<vtkActor> actor = MakeActor(polydata);
	actor->GetProperty()->SetColor(1, 0, 0);
	actor->GetProperty()->SetEdgeVisibility(true);
	ren->AddActor(actor);
	
	

	ren->ResetCamera();
    renWin->Render();
    iren->Start();

    // std::cout << "??" << std::endl;
    return 0;
}