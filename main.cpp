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
#include <vtkRendererCollection.h>
#include <vtkPointPicker.h>
#include <vtkSphereSource.h>
#include <vtkOpenGLSphereMapper.h>
#include <vtkVertex.h>
#include <vtkIntArray.h>
#include <vtkPointData.h>
#include <igl/read_triangle_mesh.h>
#include "utils.hpp"
#include <Eigen/Dense>


class InteractorStyle : public vtkInteractorStyleTrackballCamera{
public:
	static InteractorStyle* New(){return new InteractorStyle;}
	vtkTypeMacro(InteractorStyle, vtkInteractorStyleTrackballCamera);

	InteractorStyle(){}
	~InteractorStyle(){}

protected:
	bool m_propPicked= false;

	vtkSmartPointer<vtkRenderWindow> m_renWin;
	vtkSmartPointer<vtkRenderer> m_ren;
	vtkSmartPointer<vtkPolyData> m_polydata;
	vtkSmartPointer<vtkActor> m_actor;

	vtkSmartPointer<vtkPolyData> m_controlPoints;

public:
	void SetTargetPolyData(vtkSmartPointer<vtkPolyData> polydata){
		m_renWin = GetInteractor()->GetRenderWindow();
		m_ren = m_renWin->GetRenderers()->GetFirstRenderer();


		m_polydata = polydata;
		m_actor = MakeActor(polydata);
		m_actor->GetProperty()->SetColor(1, 1, 0);
		m_actor->GetProperty()->SetEdgeVisibility(true);


		this->GetInteractor()->GetPicker()->InitializePickList();
		this->GetInteractor()->GetPicker()->AddPickList(m_actor);
		this->GetInteractor()->GetPicker()->SetPickFromList(true);

		//TEMP
		// Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>> U((double*)polydata->GetPoints()->GetData()->GetVoidPointer(0), polydata->GetNumberOfPoints(), 3);	
		// U(0,0) = 10;

		m_ren->AddActor(m_actor);

		//Initialize Control Points
		m_controlPoints = vtkSmartPointer<vtkPolyData>::New();		
		m_controlPoints->SetPoints(vtkSmartPointer<vtkPoints>::New());
		m_controlPoints->SetVerts(vtkSmartPointer<vtkCellArray>::New());

		vtkNew<vtkIntArray> referencePoints;
		referencePoints->SetName("Reference");
		referencePoints->SetNumberOfComponents(1);
		m_controlPoints->GetPointData()->AddArray(referencePoints);
				
		
		vtkNew<vtkOpenGLSphereMapper> mapper;
		mapper->SetInputData(m_controlPoints);
		mapper->SetRadius(0.02);

		vtkNew<vtkActor> actor;
		actor->SetMapper(mapper);
		actor->GetProperty()->SetColor(1, 0, 0);
		m_ren->AddActor(actor);
		m_renWin->Render();

	}

protected:
	virtual void OnLeftButtonDown(){

		vtkSmartPointer<vtkPointPicker> picker = static_cast<vtkPointPicker*>(this->GetInteractor()->GetPicker());
		int* pos = this->GetInteractor()->GetEventPosition();
		bool m_propPicked = picker->Pick(pos[0], pos[1], 0, m_ren);
		

		if(m_propPicked){
			// Get Picked Position
			Eigen::RowVector3d position;
			picker->GetPickPosition(position.data());

			// Get Poitn ID
			vtkIdType pid = picker->GetPointId();			

			// Add Point
			m_controlPoints->GetPoints()->InsertNextPoint(position.data());
			m_controlPoints->GetPoints()->Modified();
			m_controlPoints->GetVerts()->InsertNextCell(vtkSmartPointer<vtkVertex>::New());
			m_controlPoints->GetPointData()->GetArray("Reference")->InsertNextTuple1(pid);

			// Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>> CU((double*)m_controlPoints->GetPoints()->GetData()->GetVoidPointer(0), m_controlPoints->GetNumberOfPoints(), 3);
			// Eigen::Map<Eigen::RowVectorXi> b((int*)m_controlPoints->GetPointData()->GetArray("Reference")->GetVoidPointer(0), m_controlPoints->GetNumberOfPoints() );

			
			m_renWin->Render();

		}else{
			Superclass::OnLeftButtonDown();
		}		
	}

	virtual void OnMouseMove(){

		if(!m_propPicked){
			Superclass::OnMouseMove();
		}			
	}

	virtual void OnLeftButtonUp(){
		m_propPicked = false;

		Superclass::OnLeftButtonUp();

	}
};

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
	iren->SetPicker(vtkSmartPointer<vtkPointPicker>::New());
    vtkNew<vtkRenderWindow> renWin;
    renWin->SetSize(1000, 1000);
    iren->SetRenderWindow(renWin);
    vtkNew<vtkRenderer> ren;
    renWin->AddRenderer(ren);
	ren->SetGradientBackground(true);
	ren->SetBackground(.2, .2, .2);
	ren->SetBackground2(.9, .9, .9);


	vtkSmartPointer<vtkPolyData> polydata = ReadPolyData(input_file);

	
	vtkNew<InteractorStyle> controller;
	iren->SetInteractorStyle(controller);
	controller->SetTargetPolyData(polydata);

	ren->ResetCamera();
    renWin->Render();
    iren->Start();

    // std::cout << "??" << std::endl;
    return 0;
}