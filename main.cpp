#include <iostream>
#include <fstream>
#include <string>
#include <vtkSmartPointer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkeigen/eigen/Dense>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkPolyDataReader.h>
#include <vtkPoints.h>
#include <vtkOpenGLSphereMapper.h>
#include <vtkProperty.h>
#include <vtkTriangle.h>
#include <vtkCellArray.h>
#include <random>
#include <vtkPLYReader.h>
#include <vtkQuadricDecimation.h>

template <typename DerivedV, typename DerivedF>
vtkSmartPointer<vtkPolyData> MakePolyData(	Eigen::PlainObjectBase<DerivedV>& V, Eigen::PlainObjectBase<DerivedF>& F){	
	vtkNew<vtkPoints> points;
	for(int vid=0 ; vid < V.rows() ; vid++){
		points->InsertNextPoint(V.coeff(vid, 0), V.coeff(vid, 1), V.coeff(vid, 2));
	}

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


template <typename DerivedV, typename DerivedF>
inline int readOFF(
	const char * str,
	Eigen::PlainObjectBase<DerivedV>& V,
	Eigen::PlainObjectBase<DerivedF>& F)
{
	std::ifstream iFile(str);
	if (!iFile) {
        std::cerr << "not a file" << std::endl;
        return 1;
    }
	std::string fT;
	char ch;
	int nV, nF, nN;
	iFile >> fT;

	if (fT.compare("OFF") != 0 && fT.compare("off") != 0) {
        std::cerr << "compare off failed" << std::endl;
        return 2;
    }
	iFile >> nV >> nF >> nN;

	V.setConstant(nV, 3, 0);
	for (int i = 0; i < nV; i++)
		iFile >> V(i, 0) >> V(i, 1) >> V(i, 2);

	F.setConstant(nF, 3, 0);
	for (int i = 0; i < nF; i++)
		iFile >> ch >> F(i, 0) >> F(i, 1) >> F(i, 2);

	return 0;
}




int main(int argc, char *argv[]){

    // if (argc < 2) {
	// 	std::cout << "lack of mesh file and result file!\n";
	// 	std::cout << "run this program in format: test_FPS meshfile resultfile \n";
	// 	return 1;
	// }

	// Eigen::MatrixXd V;
    // Eigen::MatrixXi F;
	// if (readOFF(argv[1], V, F) != 0) {
	// 	std::cout << "read the file " << argv[1] << " failed";
	// 	return 2;
	// }

	// Initialize Renderer
    vtkNew<vtkRenderWindowInteractor> iren;
    iren->SetInteractorStyle(vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New());
    vtkNew<vtkRenderWindow> renWin;
    renWin->SetSize(1000, 1000);
    iren->SetRenderWindow(renWin);
    vtkNew<vtkRenderer> ren;
    renWin->AddRenderer(ren);

	// Make Polydata and actor for rendering
	vtkNew<vtkPLYReader> reader;
	reader->SetFileName("../sample_faust.ply");
	reader->Update();

	// vtkSmartPointer<vtkPolyData> polydata = MakePolyData(V, F);
	vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
	vtkSmartPointer<vtkActor> actor = MakeActor(polydata);
	actor->GetProperty()->SetColor(1, 0, 0);
	ren->AddActor(actor);


	//TODO : Decimate Plydata
	vtkNew<vtkQuadricDecimation> decimate;
	decimate->SetInputData(polydata);
	// decimate->AttributeErrorMetricOn();
	decimate->SetTargetReduction(.75);
	// decimate->VolumePreservationOn();
	decimate->Update();


	vtkSmartPointer<vtkPolyData> polydata_downsampled = decimate->GetOutput();
	vtkSmartPointer<vtkActor> actor_downsampled = MakeActor(polydata_downsampled);
	actor_downsampled->SetPosition(1, 0, 0);
	actor_downsampled->GetProperty()->SetRepresentationToWireframe();
	ren->AddActor(actor_downsampled);
	

	

	ren->ResetCamera();
    renWin->Render();
    iren->Start();

    // std::cout << "??" << std::endl;
    return 0;
}