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
#include <vtkUnstructuredGridAlgorithm.h>
#include <vtkXMLUnstructuredGridReader.h>
#include <vtkCleanPolyData.h>
#include <vtkCleanUnstructuredGrid.h>
#include <vtkUnstructuredGridReader.h>

#include <igl/point_mesh_squared_distance.h>
#include <igl/biharmonic_coordinates.h>
#include <igl/remove_unreferenced.h>
#include <igl/volume.h>

#include "utils.hpp"
#include "SimulatorInteractor.hpp"


Eigen::MatrixXd ComputeBiharmonic(vtkSmartPointer<vtkUnstructuredGrid> high, vtkSmartPointer<vtkPolyData> low){

	Eigen::MatrixXd low_v, high_v;
	Eigen::MatrixXi high_t;

	// Extract vertices and tetrahedra from VTK data structures
	GetVertices(low, low_v);
	GetVertices(high, high_v);
	GetTetras(high, high_t);

	// --- 💡 디버깅 단계 추가 시작 💡 ---

	std::cout << high->GetNumberOfPoints() << "," << high->GetNumberOfCells() << std::endl;
	std::cout << high_v.rows() << "," << high_t.rows() << std::endl;

    // 1. 입력 데이터 기본 검사
    if (high_v.rows() == 0 || high_t.rows() == 0) {
        std::cerr << "오류: 입력 메쉬가 비어있습니다." << std::endl;
        return Eigen::MatrixXd(); // 빈 행렬 반환
    }

    // 2. 테트라헤드론 인덱스 유효성 검사
    if (high_t.maxCoeff() >= high_v.rows()) {
        std::cerr << "오류: high_t에 잘못된 정점 인덱스가 포함되어 있습니다. Max index: " << high_t.maxCoeff() << ", Vertices: " << high_v.rows() << std::endl;
        return Eigen::MatrixXd();
    }

    // 3. 퇴화된 테트라헤드론 검사 (가장 중요)
    Eigen::VectorXd vol;
    igl::volume(high_v, high_t, vol);
    const double epsilon = 1e-12; // 매우 작은 양수 값
    int degenerate_count = 0;
    for (int i = 0; i < vol.size(); ++i) {
        if (vol(i) <= epsilon) {
            degenerate_count++;
        }
    }
    if (degenerate_count > 0) {
        std::cerr << "경고: 부피가 0 또는 음수인 퇴화된 테트라헤드론이 " << degenerate_count << "개 발견되었습니다." << std::endl;
        // 여기서 프로그램을 중단시키거나, 해당 요소를 제거하는 전처리 과정이 필요할 수 있습니다.
        // return Eigen::MatrixXd(); // 문제가 될 수 있으므로 일단 종료
    }

    // --- 디버깅 단계 추가 끝 ---
	
	// Find the closest high-res vertices to the low-res vertices to use as handles
	Eigen::VectorXi b;    
	Eigen::VectorXi J = Eigen::VectorXi::LinSpaced(high_v.rows(),0,high_v.rows()-1);
	Eigen::VectorXd sqrD;
	Eigen::MatrixXd _2;
	igl::point_mesh_squared_distance(low_v,high_v,J,sqrD,b,_2);

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
		input_filename = "../resources/octopus.vtu";
	}else{
		input_filename = argv[1];
	}
	

	//Fixme :: something worng
	vtkSmartPointer<vtkXMLUnstructuredGridReader> reader = vtkSmartPointer<vtkXMLUnstructuredGridReader>::New();
	reader->SetFileName(input_filename.c_str());
	vtkSmartPointer<vtkCleanUnstructuredGrid> cleaner = vtkSmartPointer<vtkCleanUnstructuredGrid>::New();
	cleaner->SetInputConnection(reader->GetOutputPort());
	cleaner->RemovePointsWithoutCellsOff();
	cleaner->Update();


	vtkSmartPointer<vtkUnstructuredGrid> mesh = cleaner->GetOutput();

	std::vector<vtkSmartPointer<vtkUnstructuredGrid>> parts = ExtractParts(mesh);

	// Select first part
	vtkSmartPointer<vtkUnstructuredGrid> high = parts[0];
	
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