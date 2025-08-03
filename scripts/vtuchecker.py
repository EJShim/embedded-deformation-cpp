import vtk

def makeActor(inputdata):
    mapper = vtk.vtkDataSetMapper()
    mapper.SetInputData(inputdata)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    return actor


def extract_parts(input_mesh : vtk.vtkUnstructuredGrid):

    # 2. vtkConnectivityFilter 적용하여 연결된 영역 찾기
    connectivity_filter = vtk.vtkConnectivityFilter()
    connectivity_filter.SetInputData(mesh)
    # 모든 연결된 영역을 추출하도록 설정합니다.
    connectivity_filter.SetExtractionModeToAllRegions() 
    # 각 영역에 고유한 스칼라 값(RegionId)을 할당합니다.
    connectivity_filter.ColorRegionsOn() 
    connectivity_filter.Update()

    # RegionId가 추가된 결과 메쉬를 가져옵니다.
    connected_mesh = connectivity_filter.GetOutput()


    # 3. 분리된 파트의 개수 확인
    num_parts = connectivity_filter.GetNumberOfExtractedRegions()
    print(f"✅ 총 {num_parts}개의 파트로 나누어져 있습니다.")

    # 4. 각 파트를 별개의 vtkUnstructuredGrid로 분리
    part_meshes = [] # 분리된 메쉬들을 저장할 리스트
    array_name = "RegionId" # 필터가 생성한 배열 이름

    # RegionId의 최솟값, 최댓값을 가져옵니다 (보통 0부터 시작).
    min_id, max_id = connected_mesh.GetCellData().GetArray(array_name).GetRange()
    min_id, max_id = int(min_id), int(max_id)

    for i in range(min_id, max_id + 1):
        threshold = vtk.vtkThreshold()
        threshold.SetInputData(connected_mesh)
        # 'RegionId' 배열을 기준으로 셀을 필터링합니다.
        threshold.SetInputArrayToProcess(0, 0, 0, vtk.vtkDataObject.FIELD_ASSOCIATION_CELLS, array_name)
        
        # --- THIS IS THE CORRECTED PART ---
        # 현재 ID(i)에 해당하는 값만 추출하도록 상한과 하한을 설정합니다.
        threshold.SetLowerThreshold(i)
        threshold.SetUpperThreshold(i)
        # ------------------------------------

        threshold.Update()
        
        # 추출된 결과(vtkUnstructuredGrid)를 복사하여 리스트에 추가합니다.
        part_mesh = vtk.vtkUnstructuredGrid()
        part_mesh.DeepCopy(threshold.GetOutput())
        
        # 셀이 하나 이상 있는 유효한 파트만 추가합니다.
        if part_mesh.GetNumberOfCells() > 0:
            part_meshes.append(part_mesh)
            print(f"  - 파트 ID {i}: 셀 {part_mesh.GetNumberOfCells()}개 추출 완료")
            
    print(f"\n총 {len(part_meshes)}개의 vtkUnstructuredGrid가 'part_meshes' 리스트에 저장되었습니다.")

    return part_meshes

if __name__ == "__main__":

    reader = vtk.vtkXMLUnstructuredGridReader()
    reader.SetFileName("resources/compressor.vtu")
    reader.Update()

    mesh = reader.GetOutput()

    part_meshes = extract_parts(mesh)

    # Render

    iren = vtk.vtkRenderWindowInteractor()
    iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
    renWin = vtk.vtkRenderWindow()
    iren.SetRenderWindow(renWin)
    ren = vtk.vtkRenderer()
    renWin.AddRenderer(ren)



    for part in part_meshes:
        actor = makeActor(part)

        ren.AddActor(actor)
    ren.ResetCamera()
    renWin.Render()
    iren.Start()
