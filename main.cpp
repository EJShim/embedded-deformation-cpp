#include <iostream>
#include <fstream>
#include <string>
#include <vtkSmartPointer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkeigen/eigen/Dense>

#include <fast_marching.h>



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
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    if (argc < 2) {
		std::cout << "lack of mesh file and result file!\n";
		std::cout << "run this program in format: test_FPS meshfile resultfile \n";
		return 1;
	}
	if (readOFF(argv[1], V, F) != 0) {
		std::cout << "read the file " << argv[1] << " failed";
		return 2;
	}

    std::vector<int> start_points;

    FastMarchingData fmdata;
    fmdata.option.iter_max = 1000;


    
    // vtkNew<vtkRenderWindowInteractor> iren;
    // iren->SetInteractorStyle(vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New());
    // vtkNew<vtkRenderWindow> renWin;
    // renWin->SetSize(1000, 1000);
    // iren->SetRenderWindow(renWin);
    // vtkNew<vtkRenderer> ren;
    // renWin->AddRenderer(ren);

    // std::cout << "Hell World" << std::endl;

    // renWin->Render();
    // iren->Initialize();
    // iren->Start();

    // std::cout << "??" << std::endl;
    return 0;
}