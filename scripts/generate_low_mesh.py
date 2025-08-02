import vtk
import argparse
import os

def read_vtu(vtu_filename):
    """
    Reads a .vtu file and returns a vtkUnstructuredGrid.
    """
    if not os.path.exists(vtu_filename):
        print(f"Error: File not found at {vtu_filename}")
        return None
    
    reader = vtk.vtkXMLUnstructuredGridReader()
    reader.SetFileName(vtu_filename)
    reader.Update()
    
    ugrid = reader.GetOutput()
    print(f"Read {vtu_filename}: {ugrid.GetNumberOfPoints()} points, {ugrid.GetNumberOfCells()} cells.")
    return ugrid

def make_low_mesh(ugrid):
    """
    Converts a vtkUnstructuredGrid to a simplified vtkPolyData with 400-500 vertices.
    """
    if not ugrid or ugrid.GetNumberOfCells() == 0:
        print("Error: Input grid for make_low_mesh is invalid.")
        return None

    # 1. Extract the outer surface of the unstructured grid to get a vtkPolyData.
    surface_filter = vtk.vtkGeometryFilter()
    surface_filter.SetInputData(ugrid)
    surface_filter.Update()

    # 2. Ensure the surface is composed of triangles for the decimator.
    triangle_filter = vtk.vtkTriangleFilter()
    triangle_filter.SetInputData(surface_filter.GetOutput())
    triangle_filter.Update()
    
    polydata_to_decimate = triangle_filter.GetOutput()
    initial_points = polydata_to_decimate.GetNumberOfPoints()
    initial_polys = polydata_to_decimate.GetNumberOfPolys()
    print(f"Surface mesh has {initial_points} points and {initial_polys} polygons.")

    if initial_polys == 0:
        print("Error: No polygons found on the surface. Cannot decimate.")
        return polydata_to_decimate

    # 3. Decimate the mesh. For triangle meshes, #Faces ≈ 2 * #Vertices.
    # To get 400-500 vertices, we target ~900 faces.
    target_polys = 900.0
    if initial_polys > target_polys:
        reduction = (initial_polys - target_polys) / initial_polys
    else:
        reduction = 0.0

    decimator = vtk.vtkDecimatePro()
    decimator.SetInputData(polydata_to_decimate)
    decimator.SetTargetReduction(reduction)
    decimator.PreserveTopologyOn()
    decimator.Update()
    
    low_polydata = decimator.GetOutput()
    print(f"Decimated to {low_polydata.GetNumberOfPoints()} points and {low_polydata.GetNumberOfPolys()} polygons.")

    return low_polydata

def make_actor(dataset):
    """Creates a VTK actor from a dataset (ugrid or polydata)."""
    if not dataset:
        return None
    mapper = vtk.vtkDataSetMapper()
    mapper.SetInputData(dataset)

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    return actor

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Read a .vtu file, create a low-poly version, and display both.")
    parser.add_argument("input_vtu", help="Path to the input .vtu file.")
    args = parser.parse_args()

    # --- Read and Process Meshes ---
    high_mesh = read_vtu(args.input_vtu)
    if not high_mesh:
        exit()
        
    low_mesh = make_low_mesh(high_mesh)
    if not low_mesh:
        exit()

    # --- Create Actors ---
    high_actor = make_actor(high_mesh)
    low_actor = make_actor(low_mesh)

    # --- Position and Style Actors ---
    bounds = high_mesh.GetBounds()
    x_length = bounds[1] - bounds[0]
    low_actor.SetPosition(x_length * 1.1, 0, 0)
    
    low_actor.GetProperty().SetColor(1, 1, 0)  # Yellow
    low_actor.GetProperty().EdgeVisibilityOn()

    # --- VTK Rendering Setup ---
    ren = vtk.vtkRenderer()
    ren.AddActor(high_actor)
    ren.AddActor(low_actor)
    ren.SetBackground(0.1, 0.2, 0.4)
    ren.ResetCamera()

    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)
    renWin.SetSize(1200, 600)

    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)
    iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())

    renWin.Render()
    iren.Start()
