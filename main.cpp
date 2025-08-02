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
	decimator->Update();

	return decimator->GetOutput();
}


int main(int argc, char *argv[]){

	// std::string input_file_low;
	std::string input_file_high;
	// // if(argc == 1){		
	// input_file_low = "../resources/octopus-low.mesh";
	input_file_high = "../resources/octopus-high.mesh";
	
	Eigen::MatrixXd low_v, high_v, low_v_s, high_v_s;
	Eigen::MatrixXi low_f, high_f;
	Eigen::MatrixXi low_t, high_t;		
	// igl::readMESH(input_file_low, low_v, low_t, low_f);
	// std::cout << "Low-res mesh vertices: " << low_v.rows() << std::endl;

	// // Reseerve Only referenced
	Eigen::VectorXi I,J;
	// igl::remove_unreferenced(low_v.rows(),low_f,I,J);
    // std::for_each(low_f.data(),low_f.data()+low_f.size(),[&I](int & a){a=I(a);});
    // igl::slice(Eigen::MatrixXd(low_v),J,1,low_v_s);	
	
	
	igl::readMESH(input_file_high, high_v, high_t, high_f);
	igl::remove_unreferenced(high_v.rows(),high_t,I,J);
    std::for_each(high_t.data(),high_t.data()+high_t.size(),[&I](int & a){a=I(a);});
    igl::slice(Eigen::MatrixXd(high_v),J,1,high_v_s);	

	// vtkSmartPointer<vtkPolyData> polydata = MakePolyData(low_v_s, low_f);


	vtkSmartPointer<vtkUnstructuredGrid> high = MakeUnstructuredGrid(high_v_s, high_t);

	//Fixme :: something worng
	// vtkSmartPointer<vtkXMLUnstructuredGridReader> reader = vtkSmartPointer<vtkXMLUnstructuredGridReader>::New();
	// reader->SetFileName(input_file_high.c_str());
	// reader->Update();
	// vtkSmartPointer<vtkUnstructuredGrid> high = reader->GetOutput();
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