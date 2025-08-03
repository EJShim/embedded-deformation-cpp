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
#include <vtkUnstructuredGrid.h>
#include <vtkTetra.h>
#include <vtkDataSetMapper.h>
#include <vtkOBJReader.h>
#include <vtkConnectivityFilter.h>
#include <vtkDataSet.h>
#include <vtkCellData.h>
#include <vtkDataSetTriangleFilter.h>
#include <vtkThreshold.h>
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
	if(ext == "obj"){
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

template <typename DerivedV, typename DerivedF>
vtkSmartPointer<vtkUnstructuredGrid> MakeUnstructuredGrid(Eigen::PlainObjectBase<DerivedV>& V, Eigen::PlainObjectBase<DerivedF>& T) {
	vtkNew<vtkPoints> points;
	for(int vid = 0; vid < V.rows(); ++vid) {
		points->InsertNextPoint(V(vid, 0), V(vid, 1), V(vid, 2));
	}

	vtkNew<vtkUnstructuredGrid> ugrid;
	ugrid->SetPoints(points);

	for(int tid = 0; tid < T.rows(); ++tid) {
		vtkNew<vtkTetra> tetra;
		tetra->GetPointIds()->SetId(0, T(tid, 0));
		tetra->GetPointIds()->SetId(1, T(tid, 1));
		tetra->GetPointIds()->SetId(2, T(tid, 2));
		tetra->GetPointIds()->SetId(3, T(tid, 3));
		ugrid->InsertNextCell(tetra->GetCellType(), tetra->GetPointIds());
	}

	return ugrid;
}


vtkSmartPointer<vtkActor> MakeActor(vtkSmartPointer<vtkUnstructuredGrid> ugrid){

	vtkNew<vtkDataSetMapper> mapper;
	mapper->SetInputData(ugrid);

	vtkNew<vtkActor> actor;
	actor->SetMapper(mapper);


	return actor;
}


// Function to extract vertices from a VTK data structure into an Eigen matrix
void GetVertices(vtkSmartPointer<vtkPointSet> data, Eigen::MatrixXd& V)
{
    V.resize(data->GetNumberOfPoints(), 3);
    for (vtkIdType i = 0; i < data->GetNumberOfPoints(); i++)
    {
        double p[3];
        data->GetPoint(i, p);
        V.row(i) << p[0], p[1], p[2];
    }
}

// Function to extract tetrahedra from a vtkUnstructuredGrid into an Eigen matrix
void GetTetras(vtkSmartPointer<vtkUnstructuredGrid> ugrid, Eigen::MatrixXi& T)
{
    vtkCellArray* cells = ugrid->GetCells();    

    std::cout << "정보: 입력 메쉬를 4면체로 변환합니다..." << std::endl;
    
    vtkNew<vtkDataSetTriangleFilter> tetraFilter;
    tetraFilter->SetInputData(ugrid);
    tetraFilter->Update();

    // 필터의 출력을 sourceMesh로 사용합니다. 안전하게 다운캐스팅합니다.
    auto sourceMesh = vtkUnstructuredGrid::SafeDownCast(tetraFilter->GetOutput());

    // 3. 소스 메쉬(원본 또는 변환된 메쉬)에서 4면체 정보를 추출합니다.
    cells = sourceMesh->GetCells();
    

    std::cout << "Conversion Done" << std::endl;

    T.resize(cells->GetNumberOfCells(), 4);
    cells->InitTraversal();

    vtkNew<vtkIdList> ids;
    vtkIdType cellId = 0;
    while(cells->GetNextCell(ids))
    {
        if(ids->GetNumberOfIds() == 4) // It's a tetra
        {
            for(vtkIdType i = 0; i < ids->GetNumberOfIds(); ++i)
            {
                T(cellId, i) = ids->GetId(i);
            }
            cellId++;
        }
    }
    T.conservativeResize(cellId, 4);

    std::cout << T.rows() << std::endl;
}


std::vector<vtkSmartPointer<vtkUnstructuredGrid>> ExtractParts(vtkUnstructuredGrid* inputMesh)
{
    // 1. vtkConnectivityFilter 적용하여 연결된 영역 찾기
    vtkNew<vtkConnectivityFilter> connectivityFilter;
    connectivityFilter->SetInputData(inputMesh);
    connectivityFilter->SetExtractionModeToAllRegions(); // 모든 연결된 영역을 추출하도록 설정
    connectivityFilter->ColorRegionsOn();               // 각 영역에 고유 ID(RegionId) 할당
    connectivityFilter->Update();

    // RegionId가 추가된 결과 메쉬를 가져옵니다.
  // GetOutput() returns a base class pointer, so we must safely cast it.
    vtkUnstructuredGrid* connectedMesh = vtkUnstructuredGrid::SafeDownCast(connectivityFilter->GetOutput());
    // 2. 분리된 파트의 개수 확인
    int numParts = connectivityFilter->GetNumberOfExtractedRegions();
    std::cout << "✅ 총 " << numParts << "개의 파트로 나누어져 있습니다." << std::endl;

    // 3. 각 파트를 별개의 vtkUnstructuredGrid로 분리
    std::vector<vtkSmartPointer<vtkUnstructuredGrid>> partMeshes; // 결과를 저장할 벡터
    std::string arrayName = "RegionId"; // 필터가 생성한 배열 이름

    vtkDataArray* partArray = connectedMesh->GetCellData()->GetArray(arrayName.c_str());
    if (!partArray) {
        std::cerr << "오류: 'RegionId' 배열을 찾을 수 없습니다." << std::endl;
        return partMeshes; // 빈 벡터 반환
    }

    // RegionId의 최솟값, 최댓값을 가져옵니다.
    double range[2];
    partArray->GetRange(range);
    int minId = static_cast<int>(range[0]);
    int maxId = static_cast<int>(range[1]);

    for (int i = minId; i <= maxId; ++i)
    {
        vtkNew<vtkThreshold> threshold;
        threshold->SetInputData(connectedMesh);
        // 'RegionId' 배열을 기준으로 셀을 필터링합니다.
        threshold->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_CELLS, arrayName.c_str());
        
        // 현재 ID(i)에 해당하는 값만 추출하도록 상한과 하한을 설정합니다.
        threshold->SetLowerThreshold(i);
        threshold->SetUpperThreshold(i);
        threshold->Update();

        // 셀이 하나 이상 있는 유효한 파트만 리스트에 추가합니다.
        if (threshold->GetOutput()->GetNumberOfCells() > 0)
        {
            // 추출된 결과를 DeepCopy하여 완전히 새로운 객체로 만듭니다.
            vtkSmartPointer<vtkUnstructuredGrid> partMesh = vtkSmartPointer<vtkUnstructuredGrid>::New();
            partMesh->DeepCopy(threshold->GetOutput());
            
            partMeshes.push_back(partMesh);
            std::cout << "  - 파트 ID " << i << ": 셀 " << partMesh->GetNumberOfCells() << "개 추출 완료" << std::endl;
        }
    }

    std::cout << "\n총 " << partMeshes.size() << "개의 vtkUnstructuredGrid가 반환되었습니다." << std::endl;

    return partMeshes;
}