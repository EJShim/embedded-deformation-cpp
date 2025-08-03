
import vtk
import igl
import argparse
import numpy as np
from pathlib import Path
from vtk.util import numpy_support

def make_actor(ugrid):

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(ugrid)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    return actor

def generate_handle(ugrid):
    bounds = ugrid.GetBounds()

    handle = np.array([
        [ bounds[0], bounds[2]  , bounds[4] ],
        [ bounds[1], bounds[2]  , bounds[4] ],
        [ bounds[1], bounds[2]  , bounds[5] ],
        [ bounds[0], bounds[3]  , bounds[4] ],
        [ bounds[1], bounds[3]  , bounds[4] ],
        [ bounds[1], bounds[3]  , bounds[5] ],
        [ bounds[0], bounds[2]  , bounds[5] ],
        [ bounds[0], bounds[3]  , bounds[5] ]
    ], dtype=np.float64)
    
    return handle
    

def convert_to_triangle_mesh(unstructured_grid):
    """
    주어진 VTK UnstructuredGrid의 표면을 삼각형(only triangle)으로 이루어진 polydata로 변환.
    """
    # Step 1: UnstructuredGrid → Surface
    surface_filter = vtk.vtkDataSetSurfaceFilter()
    surface_filter.SetInputData(unstructured_grid)
    

    # Step 2: Surface → Triangulated PolyData
    triangle_filter = vtk.vtkTriangleFilter()
    triangle_filter.SetInputConnection(surface_filter.GetOutputPort())
    
    cleanFilter = vtk.vtkCleanPolyData()
    cleanFilter.SetInputConnection(triangle_filter.GetOutputPort())
    cleanFilter.Update()
    

    # 결과를 PolyData로 반환
    triangle_mesh = vtk.vtkPolyData()
    triangle_mesh.DeepCopy(cleanFilter.GetOutput())



    return triangle_mesh

def convert_to_tetrahedra(unstructured_grid):
    """
    입력으로 받은 VTK UnstructuredGrid를 tetrahedron-only grid로 변환.
    """
    # Tetrahedralize 하기 위한 필터
    tetra_filter = vtk.vtkDataSetTriangleFilter()
    tetra_filter.SetInputData(unstructured_grid)
    tetra_filter.Update()

    # 결과를 UnstructuredGrid로 변환
    output_grid = vtk.vtkUnstructuredGrid()
    output_grid.DeepCopy(tetra_filter.GetOutput())

    return output_grid


def compute_biharmonic(low_v, fine_mesh):


    # generate tet
    high_V = numpy_support.vtk_to_numpy(fine_mesh.GetPoints().GetData())



    num_cells = fine_mesh.GetNumberOfCells()
    high_connectivity = numpy_support.vtk_to_numpy(fine_mesh.GetPolys().GetData()).reshape(num_cells, 4)[:,1:]

    # Point Matching
    asdf = np.expand_dims(np.arange(high_V.shape[0]), 1)
    _, b, _ = igl.point_mesh_squared_distance(low_v, high_V, asdf)    

    S = np.expand_dims(b, 1)    
    

    W = igl.biharmonic_coordinates(high_V, high_connectivity, S, 2)

    return W


def make_pointcloud(polydata):
    mapper = vtk.vtkOpenGLSphereMapper()
    mapper.SetInputData(polydata)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(1,0,0)

    return actor

def main():

    print("Hello from biharmonic-morphing!")

    iren = vtk.vtkRenderWindowInteractor()
    iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
    renWin = vtk.vtkRenderWindow()
    iren.SetRenderWindow(renWin)
    ren = vtk.vtkRenderer()
    renWin.AddRenderer(ren)

    # Read VTK
    reader = vtk.vtkXMLUnstructuredGridReader()
    reader.SetFileName(args.vtu)
    reader.Update()
    target_data = reader.GetOutput()

    fine_mesh=convert_to_triangle_mesh(target_data)
    actor = make_actor(fine_mesh)
    ren.AddActor(actor)


    # Generate Handle by decimate    
    decimate = vtk.vtkQuadricDecimation()
    decimate.SetInputData(fine_mesh)
    decimate.SetTargetReduction(0.99)
    decimate.VolumePreservationOff()
    decimate.Update()          
    handle_poly = decimate.GetOutput()    
    handle = make_pointcloud(handle_poly)
    ren.AddActor(handle)


    
    # Generate Handle
    # handle_square = generate_handle(target_data)
    handle_v = numpy_support.vtk_to_numpy(handle_poly.GetPoints().GetData())
    

    # Compute Biharmonic
    biharmonic = compute_biharmonic(handle_v, fine_mesh)

    



    #TODO : Move handle
    
    morph_handle = vtk.vtkPolyData()
    morph_handle.DeepCopy(handle_poly)
    morph_handle_actor = make_pointcloud(morph_handle)
    morph_handle_actor.SetPosition(40, 0, 0)
    ren.AddActor(morph_handle_actor)
    

    morph_data = vtk.vtkPolyData()
    morph_data.DeepCopy(fine_mesh)
    morph_nodes = numpy_support.vtk_to_numpy(morph_data.GetPoints().GetData())


    # Morph handle
    morph_handle_nodes = numpy_support.vtk_to_numpy(morph_handle.GetPoints().GetData())

    morph_handle_nodes[0][0] += 10

    morphed = biharmonic @ morph_handle_nodes


    morph_nodes[:]  = morphed
    

    morph_actor = make_actor(morph_data)
    morph_actor.SetPosition(40,0,0)
    ren.AddActor(morph_actor)
    ren.ResetCamera()

    renWin.Render()
    iren.Start()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--vtu", type=Path, default="RUBBER.vtu")
    args = parser.parse_args()

    main()
