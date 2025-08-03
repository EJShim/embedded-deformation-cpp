#include <iostream>
#include <string>
#include <vector>
#include <vtkCellType.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTriangle.h>
#include <vtkUnstructuredGrid.h>
#include <vtkGeometryFilter.h>
#include <vtkTriangleFilter.h>
#include <vtkQuadricDecimation.h>
#include <vtkXMLUnstructuredGridReader.h>
#include <vtkCleanPolyData.h>
#include <vtkCleanUnstructuredGrid.h>
#include <vtkUnstructuredGridReader.h>

#include <igl/point_mesh_squared_distance.h>
#include <igl/biharmonic_coordinates.h>
#include <igl/remove_unreferenced.h>
#include "utils.hpp"
#include "SimulatorInteractor.hpp"


Eigen::MatrixXd ComputeBiharmonic(vtkSmartPointer<vtkUnstructuredGrid> high, vtkSmartPointer<vtkPolyData> low){

	Eigen::MatrixXd low_v, high_v;
	Eigen::MatrixXi high_t;

	// Extract vertices and tetrahedra from VTK data structures
	GetVertices(low, low_v);
	GetVertices(high, high_v);
	GetTetras(high, high_t);
	
	// Find the closest high-res vertices to the low-res vertices to use as handles
	Eigen::VectorXi b;
    {
        Eigen::VectorXi J = Eigen::VectorXi::LinSpaced(high_v.rows(),0,high_v.rows()-1);
        Eigen::VectorXd sqrD;
        Eigen::MatrixXd _2;
        igl::point_mesh_squared_distance(low_v,high_v,J,sqrD,b,_2);
    }
	std::vector<std::vector<int> > S;
    igl::matrix_to_list(b,S);
	std::cout<<"Computing weights for "<<b.size()<< " handles at "<<high_v.rows()<<" vertices..."<<std::endl;
	
	// Set the number of basis functions for biharmonic coordinates
	const int k = 2;
	Eigen::MatrixXd W;
    igl::biharmonic_coordinates(high_v, high_t, S, k, W);
	std::cout << "Computed weights matrix of size: " << W.rows() << "x" << W.cols() << std::endl;

	return W;
}


vtkSmartPointer<vtkPolyData> GenerateLowResolutionMesh(vtkSmartPointer<vtkUnstructuredGrid> highMesh){

	vtkSmartPointer<vtkGeometryFilter> surfaceFilter = vtkSmartPointer<vtkGeometryFilter>::New();
	surfaceFilter->SetInputData(highMesh);

	vtkSmartPointer<vtkTriangleFilter> trianglelFilter = vtkSmartPointer<vtkTriangleFilter>::New();
	trianglelFilter->SetInputConnection(surfaceFilter->GetOutputPort());
	trianglelFilter->Update();

	vtkSmartPointer<vtkPolyData> initialMesh  = trianglelFilter->GetOutput();
	int initialPoints = initialMesh->GetNumberOfPoints();
	int initialPolys = initialMesh->GetNumberOfPolys();

	double reduction = (initialPolys - 900.0) / initialPolys;

	vtkSmartPointer<vtkQuadricDecimation> decimator = vtkSmartPointer<vtkQuadricDecimation>::New();
	decimator->SetInputData(initialMesh);
	decimator->SetTargetReduction(reduction);

	vtkSmartPointer<vtkCleanPolyData> cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
	cleaner->SetInputConnection(decimator->GetOutputPort());
	cleaner->Update();

	return cleaner->GetOutput();
}


int main(int argc, char *argv[]){

	// std::string input_file_low;
	std::string input_filename;
	if(argc == 1){		
		input_filename = argv[1];
	}
	
	input_filename = "../resources/octopus.vtu";

	//Fixme :: something worng
	vtkSmartPointer<vtkXMLUnstructuredGridReader> reader = vtkSmartPointer<vtkXMLUnstructuredGridReader>::New();
	reader->SetFileName(input_filename.c_str());
	reader->Update();
	vtkSmartPointer<vtkUnstructuredGrid> high = reader->GetOutput();
	
	// Triangulate and decimate, generate low triangle mesh
	vtkSmartPointer<vtkPolyData> low = GenerateLowResolutionMesh(high);

	// Compute Biharmonic Weights
	Eigen::MatrixXd W = ComputeBiharmonic(high, low);
    
	// Initialize Renderer
    vtkNew<vtkRenderWindowInteractor> iren;
	iren->SetPicker(vtkSmartPointer<vtkPointPicker>::New());
    vtkNew<vtkRenderWindow> renWin;
    renWin->SetSize(1000, 1000);
    iren->SetRenderWindow(renWin);
    vtkNew<vtkRenderer> ren;
    renWin->AddRenderer(ren);
	ren->SetGradientBackground(true);
	
	// Add to system
	vtkNew<CustomInteractorStyle> controller;
	iren->SetInteractorStyle(controller);
	controller->SetTargetPolyData(low, high);
	controller->SetBiharmonicWeights(W.cast<float>());
	
	ren->ResetCamera();
    renWin->Render();
    iren->Start();

    return 0;
}