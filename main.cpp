#include <iostream>
#include <fstream>
#include <string>
#include <vtkSmartPointer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkOBJReader.h>
#include <igl/read_triangle_mesh.h>
#include "utils.hpp"
#include <Eigen/Dense>


int main(int argc, char *argv[]){

	std::string input_file;
	if(argc == 1){		
		input_file = "../resources/decimated-knight.off";
	}else{
		input_file = argv[1];
	}
	std::cout << input_file << std::endl;

    
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


	vtkSmartPointer<vtkPolyData> polydata = ReadPolyData(input_file);
	vtkSmartPointer<vtkActor> actor = MakeActor(polydata);
	actor->GetProperty()->SetColor(1, 0, 0);
	actor->GetProperty()->SetEdgeVisibility(true);
	ren->AddActor(actor);

	
	// check if changing V affects polydata modification
	// Assign V with new Eigen::matrix
	Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>> U((double*)polydata->GetPoints()->GetData()->GetVoidPointer(0), polydata->GetNumberOfPoints(), 3);	
	U(0,0) = 10;

	
	ren->ResetCamera();
    renWin->Render();
    iren->Start();

    // std::cout << "??" << std::endl;
    return 0;
}