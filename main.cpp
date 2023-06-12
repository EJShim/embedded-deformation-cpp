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
#include <igl/readMESH.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/cotmatrix.h>
#include <igl/polar_svd3x3.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cmath>
#include "utils.hpp"
#include <igl/point_mesh_squared_distance.h>
// #include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/biharmonic_coordinates.h>
#include <igl/remove_unreferenced.h>


class vtkTimerCallback : public vtkCommand
{
	public:		
	vtkTimerCallback() = default;

	igl::min_quad_with_fixed_data<float> arap_data;
	Eigen::SparseMatrix<float> arap_K;	
	vtkSmartPointer<vtkPolyData> m_polydata;
	vtkSmartPointer<vtkPolyData> m_highPolyData;
	vtkSmartPointer<vtkPolyData> m_controlPoints;
	Eigen::MatrixXf m_biharmonic_w;


	static vtkTimerCallback* New(){
		vtkTimerCallback* cb = new vtkTimerCallback;
		

		cb->TimerCount = 0;
		return cb;
	}

	virtual void Execute(vtkObject* caller, unsigned long eventId, void* vtkNotUsed(callData) ){
		if (vtkCommand::TimerEvent == eventId){
			++this->TimerCount;
		}
		
		SingleIteration();
		
		static_cast<vtkRenderWindowInteractor*>(caller)->GetRenderWindow()->Render();
	}

	void SetBiHarmonicWeights(Eigen::MatrixXf w){
		m_biharmonic_w = w;
	}

	void SetHighPolyData(vtkSmartPointer<vtkPolyData> polydtaa){
		m_highPolyData = polydtaa;
	}

	void Initialize(vtkSmartPointer<vtkPolyData> polydata, vtkSmartPointer<vtkPolyData> cotrolPoints){		
		m_polydata = polydata;
		m_controlPoints = cotrolPoints;
		

		//Extract Vertices
		Eigen::Map<Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> V((float*)polydata->GetPoints()->GetData()->GetVoidPointer(0), polydata->GetNumberOfPoints(), 3);	

		//Extract Faces
		vtkIdType num_faces = polydata->GetPolys()->GetNumberOfCells();		
		Eigen::Map<Eigen::Matrix<vtkIdType, -1, -1, Eigen::RowMajor>> F_raw((vtkIdType*)polydata->GetPolys()->GetData()->GetVoidPointer(0), num_faces ,4);	
		Eigen::Matrix<vtkIdType, -1, -1, Eigen::RowMajor> F = F_raw(Eigen::all,{1,2,3});

		// Extract Selected indices
		Eigen::Map<Eigen::VectorXi> b((int*)cotrolPoints->GetPointData()->GetArray("Reference")->GetVoidPointer(0), cotrolPoints->GetNumberOfPoints() );
		

		// cot matrix
		Eigen::SparseMatrix<float> L;  
		igl::cotmatrix(V,F,L);

		Eigen::SparseMatrix<float> Aeq;
		igl::min_quad_with_fixed_precompute(L,b,Aeq,false,arap_data);

		Eigen::MatrixXf CE;
		igl::cotmatrix_entries(V, F, CE);

		// Build K
		
		arap_K.resize(V.rows(),3*V.rows());
		
		typedef Eigen::Triplet<float> T;
		std::vector<T> tripletList;
		tripletList.reserve(F.rows()*3*6);


		for (int f=0; f<F.rows(); f++){
				for (int v=0; v<3; v++){
				// for triangle meshes
				int i = F(f, (v + 1)%3);
				int j = F(f, (v + 2)%3);
				// cot_v corresponds to the opposi₩te half edge
			
				Eigen::Vector3f eij = CE(f,v) * (V.row(i) - V.row(j));
				eij = eij/3.0;
			
				for(int k=0; k<3; k++){
					for(int t=0; t<3; t++){
					tripletList.push_back(T(i, 3*F(f,k) + t, eij(t)));
					tripletList.push_back(T(j, 3*F(f,k) + t, -eij(t)));
					}
				}      
			}
		}
		
		arap_K.setFromTriplets(tripletList.begin(), tripletList.end());
	}

	void SingleIteration(){				
		Eigen::Map<Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> CU((float*)m_controlPoints->GetPoints()->GetData()->GetVoidPointer(0), m_controlPoints->GetNumberOfPoints(), 3);
		Eigen::Map<Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> V((float*)m_polydata->GetPoints()->GetData()->GetVoidPointer(0), m_polydata->GetNumberOfPoints(), 3);	
		Eigen::Map<Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> H_V((float*)m_highPolyData->GetPoints()->GetData()->GetVoidPointer(0), m_highPolyData->GetNumberOfPoints(), 3);	
		
		Eigen::MatrixXf C = arap_K.transpose() * V;

		Eigen::MatrixXf R(3*arap_data.n, 3);
		for (int k=0; k<arap_data.n; k++){
			Eigen::Matrix3f Rk;
			Eigen::Matrix3f Ck = C.block(k*3, 0, 3, 3);
			igl::polar_svd3x3(Ck, Rk);
			R.block(k*3, 0, 3, 3) = Rk;    
	  	}

		// from min_quad_with_fixed_solve code
		Eigen::VectorXf Beq;
		Eigen::MatrixXf B = arap_K*R;

		Eigen::Matrix<float, -1, -1, Eigen::RowMajor> U = V;		
		igl::min_quad_with_fixed_solve(arap_data, B, CU, Beq, U);		

		// assign U to polydtaa??
		V = U;				
		// std::cout << m_biharmonic_w.rows() << "," << m_biharmonic_w.cols() << "," << H_V.rows() << std::endl;
		
		H_V = m_biharmonic_w * U;
		m_polydata->GetPoints()->Modified();		
		m_highPolyData->GetPoints()->Modified();		
		
	}

	private:
	int TimerCount = 0;
};


class InteractorStyle : public vtkInteractorStyleTrackballCamera{
public:
	static InteractorStyle* New(){return new InteractorStyle;}
	vtkTypeMacro(InteractorStyle, vtkInteractorStyleTrackballCamera);

	InteractorStyle(){		
	}
	~InteractorStyle(){}

protected:
	vtkIdType m_pickedId;
	double m_viewDistance = 0.5;
	bool m_bSimulation = false;
	int m_timerId = -1;

	vtkSmartPointer<vtkRenderWindow> m_renWin;
	vtkSmartPointer<vtkRenderer> m_ren;
	vtkSmartPointer<vtkPolyData> m_polydata;
	vtkSmartPointer<vtkActor> m_actor;

	vtkSmartPointer<vtkPolyData> m_hPoly;
	vtkSmartPointer<vtkActor> m_hActor;

	vtkSmartPointer<vtkPolyData> m_controlPoints;
	vtkSmartPointer<vtkActor> m_controlPointsActor;

	//Simulator
	vtkSmartPointer<vtkTimerCallback> m_Simulator;

public:
	void SetBiharmonicWeights(Eigen::MatrixXf w){
		m_Simulator->SetBiHarmonicWeights(w);
	}


	void SetTargetPolyData(vtkSmartPointer<vtkPolyData> polydata, vtkSmartPointer<vtkPolyData> h_polydata){
		this->GetInteractor()->GetPicker()->SetPickFromList(true);

		m_renWin = GetInteractor()->GetRenderWindow();
		m_ren = m_renWin->GetRenderers()->GetFirstRenderer();

		m_polydata = polydata;
		m_actor = MakeActor(polydata);
		m_actor->GetProperty()->SetColor(1, 1, 0);
		m_actor->GetProperty()->SetEdgeVisibility(true);
		m_actor->GetProperty()->SetOpacity(0.5);
		double* bounds = m_actor->GetBounds();
		double xlen = bounds[1] - bounds[0];
		double ylen = bounds[2] - bounds[3];
		double zlen = bounds[4] - bounds[5];
		double length = sqrt(xlen*xlen + ylen*ylen + zlen*zlen);
		m_ren->AddActor(m_actor);

		m_hPoly = h_polydata;
		m_hActor = MakeActor(h_polydata);
		m_hActor->SetPosition(xlen * 1.1,0,0);
		m_ren->AddActor(m_hActor);

		CalculateBiHarmonic(m_polydata, m_hPoly);

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
		mapper->SetRadius(length / 50);

		m_controlPointsActor = vtkSmartPointer<vtkActor>::New();
		m_controlPointsActor->SetMapper(mapper);
		m_controlPointsActor->GetProperty()->SetColor(1, 0, 0);
		m_controlPointsActor->GetProperty()->SetPointSize(10);
		m_ren->AddActor(m_controlPointsActor);
		m_renWin->Render();

		//Intiialize Timer		
		m_Simulator = vtkSmartPointer<vtkTimerCallback>::New();
		this->GetInteractor()->AddObserver(vtkCommand::TimerEvent, m_Simulator);

		Update();
	}

protected:

	void CalculateBiHarmonic(vtkSmartPointer<vtkPolyData> low, vtkSmartPointer<vtkPolyData> high){

		
	}

	void Update(){		

		vtkSmartPointer<vtkPointPicker> picker = static_cast<vtkPointPicker*>(this->GetInteractor()->GetPicker());

		if(m_bSimulation){
			m_ren->SetBackground(.5, .5, .9);
			m_ren->SetBackground2(.9, .9, .9);
			
			picker->InitializePickList();
			picker->AddPickList(m_controlPointsActor);			
			
			m_Simulator->Initialize(m_polydata, m_controlPoints);
			m_Simulator->SetHighPolyData(m_hPoly);
			m_renWin->Render();
			m_timerId = Interactor->CreateRepeatingTimer(10);
			Interactor->Start();

			
		}else{
			m_ren->SetBackground(.2, .2, .2);
			m_ren->SetBackground2(.9, .9, .9);
			
			picker->InitializePickList();
			picker->AddPickList(m_actor);			

			if(m_timerId != -1) this->GetInteractor()->DestroyTimer(m_timerId);
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
				CU(m_pickedId, 0) = (float)worldPos[0];
				CU(m_pickedId, 1) = (float)worldPos[1];
				CU(m_pickedId, 2) = (float)worldPos[2];
				
				m_controlPoints->GetPoints()->Modified();

				// Simulate
				m_Simulator->SingleIteration();				

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

	std::string input_file_low;
	std::string input_file_high;
	// if(argc == 1){		
	input_file_low = "../resources/vmod_low.ply";
	input_file_high = "../resources/vmod_high.ply";
	
	Eigen::MatrixXd low_v, high_v;
	Eigen::MatrixXi low_f, high_f;		
	igl::read_triangle_mesh(input_file_low, low_v, low_f);	
	igl::read_triangle_mesh(input_file_high, high_v, high_f);


		
	
	// Remove Unreferenced -  low
	Eigen::VectorXi I,J;
	igl::remove_unreferenced(low_v.rows(),low_f,I,J);
    std::for_each(low_f.data(),low_f.data()+low_f.size(),[&I](int & a){a=I(a);});
    igl::slice(Eigen::MatrixXd(low_v),J,1,low_v);		

	igl::remove_unreferenced(high_v.rows(),high_f,I,J);
    std::for_each(high_f.data(),high_f.data()+high_f.size(),[&I](int & a){a=I(a);});
	igl::slice(Eigen::MatrixXd(high_v),J,1,high_v);		

	
	
	
	// Calculate W here, because it is too 귀찮음 
	Eigen::VectorXi b;
    {

        Eigen::VectorXi J = Eigen::VectorXi::LinSpaced(high_v.rows(),0,high_v.rows()-1);
        Eigen::VectorXd sqrD;
        Eigen::MatrixXd _2;
        igl::point_mesh_squared_distance(low_v,high_v,J,sqrD,b,_2);
    }
	// igl::slice(high_v,b,1,low_v);


	std::vector<std::vector<int> > S;
    igl::matrix_to_list(b,S);
	cout<<"Computing weights for "<<b.size()<< " handles at "<<high_v.rows()<<" vertices..."<<endl;
	const int k = 2;
	Eigen::MatrixXd W;
    igl::biharmonic_coordinates(high_v, high_f,S,k,W);
	std::cout << W.rows() << "," << W.cols() << std::endl;



	// // Remove Unreferenced - high
	// igl::remove_unreferenced(high_v.rows(),high_f,I,J);
    // std::for_each(high_f.data(),high_f.data()+high_f.size(),[&I](int & a){a=I(a);});
	// // std::for_each(b.data(),b.data()+b.size(),[&I](int & a){a=I(a);});

    // igl::slice(Eigen::MatrixXd(high_v),J,1,high_v);		
	// igl::slice(Eigen::MatrixXd(W),J,1,W);

	

	

	// Test TEtgen hihg
	// Tetrahedralized interior
	// Eigen::MatrixXd TV;
	// Eigen::MatrixXi TT;
	// Eigen::MatrixXi TF;
	// igl::copyleft::tetgen::tetrahedralize(high_v, high_f, "pq1.414Y", TV,TT,TF);
	// std::cout << TV.rows() << "," << TT.rows() << "," << TF.rows() << std::endl;
	


	
    
	// Initialize Renderer
    vtkNew<vtkRenderWindowInteractor> iren;
	iren->SetPicker(vtkSmartPointer<vtkPointPicker>::New());
    vtkNew<vtkRenderWindow> renWin;
    renWin->SetSize(1000, 1000);
    iren->SetRenderWindow(renWin);
    vtkNew<vtkRenderer> ren;
    renWin->AddRenderer(ren);
	ren->SetGradientBackground(true);

	//Read Polydata
	vtkSmartPointer<vtkPolyData> polydata = MakePolyData(low_v, low_f);
	vtkSmartPointer<vtkPolyData> h_polydata = MakePolyData(high_v, high_f);

	std::cout << polydata->GetNumberOfPoints() << std::endl;

	// Add to system
	vtkNew<InteractorStyle> controller;
	iren->SetInteractorStyle(controller);
	controller->SetTargetPolyData(polydata, h_polydata);
	controller->SetBiharmonicWeights(W.cast<float>());
	
	ren->ResetCamera();
    renWin->Render();
    iren->Start();

    return 0;
}