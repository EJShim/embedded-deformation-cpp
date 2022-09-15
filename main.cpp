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
#include <vtkIdList.h>
#include <vtkPointData.h>
#include <vtkCoordinate.h>
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
	vtkIdType m_pickedId;
	double m_viewDistance = 0.5;
	bool m_bSimulation = false;

	vtkSmartPointer<vtkRenderWindow> m_renWin;
	vtkSmartPointer<vtkRenderer> m_ren;
	vtkSmartPointer<vtkPolyData> m_polydata;
	vtkSmartPointer<vtkActor> m_actor;

	vtkSmartPointer<vtkPolyData> m_controlPoints;
	vtkSmartPointer<vtkActor> m_controlPointsActor;

public:
	void SetTargetPolyData(vtkSmartPointer<vtkPolyData> polydata){
		this->GetInteractor()->GetPicker()->SetPickFromList(true);

		m_renWin = GetInteractor()->GetRenderWindow();
		m_ren = m_renWin->GetRenderers()->GetFirstRenderer();

		m_polydata = polydata;
		m_actor = MakeActor(polydata);
		m_actor->GetProperty()->SetColor(1, 1, 0);
		m_actor->GetProperty()->SetEdgeVisibility(true);
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

		m_controlPointsActor = vtkSmartPointer<vtkActor>::New();
		m_controlPointsActor->SetMapper(mapper);
		m_controlPointsActor->GetProperty()->SetColor(1, 0, 0);
		m_controlPointsActor->GetProperty()->SetPointSize(10);
		m_ren->AddActor(m_controlPointsActor);
		m_renWin->Render();

		Update();
	}

protected:
	void Update(){

		vtkSmartPointer<vtkPointPicker> picker = static_cast<vtkPointPicker*>(this->GetInteractor()->GetPicker());

		if(m_bSimulation){
			m_ren->SetBackground(.5, .5, .9);
			m_ren->SetBackground2(.9, .9, .9);
			
			picker->InitializePickList();
			picker->AddPickList(m_controlPointsActor);			

		}else{
			m_ren->SetBackground(.2, .2, .2);
			m_ren->SetBackground2(.9, .9, .9);
			
			picker->InitializePickList();
			picker->AddPickList(m_actor);	
		}

		m_renWin->Render();
	}

	virtual void OnLeftButtonDown(){

		vtkSmartPointer<vtkPointPicker> picker = static_cast<vtkPointPicker*>(this->GetInteractor()->GetPicker());
		int* pos = this->GetInteractor()->GetEventPosition();
		picker->Pick(pos[0], pos[1], 0, m_ren);		
		m_pickedId = picker->GetPointId();					


		if(m_pickedId >= 0){
			// Get Picked Position
			Eigen::RowVector3d position;
			picker->GetPickPosition(position.data());


			if(!m_bSimulation){		
				// Add Point
				vtkIdType id = m_controlPoints->GetNumberOfPoints();
				m_controlPoints->GetPoints()->InsertNextPoint(position.data());
				m_controlPoints->GetPoints()->Modified();
				vtkNew<vtkVertex> vertex;
				vertex->GetPointIds()->SetId(0, id);
				m_controlPoints->GetVerts()->InsertNextCell(vertex);
				m_controlPoints->GetPointData()->GetArray("Reference")->InsertNextTuple1(m_pickedId);
				m_controlPointsActor->Modified();
			}else{
				
				m_ren->SetWorldPoint(position.data());
				m_ren->WorldToView();
				Eigen::RowVector3d viewPos(m_ren->GetViewPoint());
				m_viewDistance = viewPos[2];
			}


			// Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>> CU((double*)m_controlPoints->GetPoints()->GetData()->GetVoidPointer(0), m_controlPoints->GetNumberOfPoints(), 3);
			// Eigen::Map<Eigen::RowVectorXi> b((int*)m_controlPoints->GetPointData()->GetArray("Reference")->GetVoidPointer(0), m_controlPoints->GetNumberOfPoints() );

			
			m_renWin->Render();

		}else{
			Superclass::OnLeftButtonDown();
		}		
	}

	virtual void OnMouseMove(){

		// Eigen test
		Eigen::Map<Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> CU((float*)m_controlPoints->GetPoints()->GetData()->GetVoidPointer(0), m_controlPoints->GetNumberOfPoints(), 3);

		if(m_pickedId == -1){
			Superclass::OnMouseMove();
		}else{
			if(m_bSimulation){
				// Move Control Points

				Eigen::RowVector3d position(m_controlPoints->GetPoint(m_pickedId));
				Eigen::RowVector2i screenPos(this->GetInteractor()->GetEventPosition());
					
				m_ren->SetDisplayPoint(screenPos[0], screenPos[1], 0);
				m_ren->DisplayToView();				
				Eigen::RowVector3d viewPos(m_ren->GetViewPoint());
				m_ren->SetViewPoint(viewPos[0], viewPos[1], m_viewDistance);
				m_ren->ViewToWorld();
				Eigen::RowVector3d worldPos(m_ren->GetWorldPoint());
								
				// m_controlPoints->GetPoints()->SetPoint(m_pickedId, worldPos.data());	
				CU(m_pickedId, 0) = worldPos[0];
				CU(m_pickedId, 1) = worldPos[1];
				CU(m_pickedId, 2) = worldPos[2];
				
				m_controlPoints->GetPoints()->Modified();
				
				m_renWin->Render();
			}
		}
	}

	virtual void OnLeftButtonUp(){
		m_pickedId = -1;

		Superclass::OnLeftButtonUp();
	}

	virtual void OnKeyDown(){

		char keycode = this->GetInteractor()->GetKeyCode();

		if( keycode == 32 ){
			m_bSimulation = !m_bSimulation;
			Update();
		}

		Superclass::OnKeyDown();
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