import vtk
import igl
import numpy as np
from vtk.util import numpy_support
import argparse
import os

def build_tetra(v, t):

    points = vtk.vtkPoints()
    points.SetData(numpy_support.numpy_to_vtk(v))

    results = vtk.vtkUnstructuredGrid()
    results.SetPoints(points)

    # Set Tetrahedra connectivity
    num_tetras = t.shape[0]
    # The connectivity array for VTK needs to be in the format:
    # [n_points_cell_1, p1_1, p1_2, ..., n_points_cell_2, p2_1, p2_2, ...]
    # For tetrahedra, n_points is always 4.
    connectivity = np.hstack((np.full((num_tetras, 1), 4, dtype=t.dtype), t)).ravel()

    cells = vtk.vtkCellArray()
    cells.SetCells(num_tetras, numpy_support.numpy_to_vtkIdTypeArray(connectivity, deep=True))

    results.SetCells(vtk.VTK_TETRA, cells)

    return results

def make_actor(ugrid):
    mapper = vtk.vtkDataSetMapper()
    mapper.SetInputData(ugrid)

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    return actor

def save_vtu(ugrid, filename):
    """Saves a vtkUnstructuredGrid to a .vtu file."""
    writer = vtk.vtkXMLUnstructuredGridWriter()
    writer.SetFileName(filename)
    writer.SetInputData(ugrid)
    writer.Write()
    print(f"Successfully saved {filename}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert a .mesh file to a .vtu file and render it.")
    parser.add_argument("input_mesh", help="Path to the input .mesh file.")
    args = parser.parse_args()

    input_path = args.input_mesh
    
    if not os.path.exists(input_path):
        print(f"Error: Input file not found at {input_path}")
        exit()

    output_path = os.path.splitext(input_path)[0] + '.vtu'

    v, t, f = igl.readMESH(input_path)
    
    ugrid = build_tetra(v, t)

    save_vtu(ugrid, output_path)

    # --- VTK Rendering ---
    iren = vtk.vtkRenderWindowInteractor()
    iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
    renWin = vtk.vtkRenderWindow()
    iren.SetRenderWindow(renWin)
    ren = vtk.vtkRenderer()
    renWin.AddRenderer(ren)

    actor = make_actor(ugrid)

    ren.AddActor(actor)
    ren.ResetCamera()

    renWin.Render()
    iren.Start()
