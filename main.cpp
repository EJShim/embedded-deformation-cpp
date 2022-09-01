#include <iostream>
#include <vtkSmartPointer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkInteractorStyleTrackballCamera.h>

int main(){
    vtkNew<vtkRenderWindowInteractor> iren;
    iren->SetInteractorStyle(vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New());
    vtkNew<vtkRenderWindow> renWin;
    renWin->SetSize(1000, 1000);
    iren->SetRenderWindow(renWin);
    vtkNew<vtkRenderer> ren;
    renWin->AddRenderer(ren);

    std::cout << "Hell World" << std::endl;

    renWin->Render();
    iren->Initialize();
    iren->Start();

    std::cout << "??" << std::endl;
    return 0;
}