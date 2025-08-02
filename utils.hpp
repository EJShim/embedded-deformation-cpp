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
    T.resize(ugrid->GetNumberOfCells(), 4);
    ugrid->GetCells()->InitTraversal();
    vtkNew<vtkIdList> ids;
    vtkIdType cellId = 0;
    while(ugrid->GetCells()->GetNextCell(ids))
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
}
