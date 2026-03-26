"""rmsh Python API usage examples.

This script demonstrates basic usage of the rmsh Python bindings.
Note: Running this script requires the compiled _rmsh native extension to be
installed or available on sys.path (e.g., after `maturin develop`).

Examples covered:
  1. Load a STEP file, generate a surface mesh, write to .msh
  2. Load a .msh file and open it in the viewer
  3. Generate a 2-D polygon mesh from scratch and view it
"""

import os
import sys

# If running from the project root without installing, add the build output dir.
# Adjust the path to wherever maturin places the compiled .pyd / .so.
# sys.path.insert(0, "target/debug")

import rmsh

# ---------------------------------------------------------------------------
# Example 1: STEP → mesh → .msh file
# ---------------------------------------------------------------------------
def example_step_to_msh(step_path: str, output_path: str = "output.msh") -> None:
    """Load a STEP file, generate a 3-D surface mesh, and write it out."""
    rmsh.initialize()

    rmsh.open(step_path)

    # Optionally tune meshing parameters via options
    rmsh.option.setNumber("Mesh.MeshSizeFactor", 1.0)

    # Generate a dimension-3 surface mesh
    rmsh.model.mesh.generate(3)

    rmsh.write(output_path)
    print(f"Wrote mesh to {output_path}")

    rmsh.finalize()


# ---------------------------------------------------------------------------
# Example 2: Load an existing .msh and inspect it
# ---------------------------------------------------------------------------
def example_load_msh(msh_path: str) -> None:
    """Load a .msh file and open it in the interactive viewer."""
    rmsh.initialize()

    rmsh.open(msh_path)

    # Launch the native 3-D viewer (blocks until the window is closed)
    rmsh.gui.initialize()
    rmsh.gui.run()

    rmsh.finalize()


# ---------------------------------------------------------------------------
# Example 3: Build a 2-D polygon mesh in memory and view it
# ---------------------------------------------------------------------------
def example_2d_polygon_mesh() -> None:
    """Generate a 2-D triangulation of a square polygon and view it."""
    rmsh.initialize()

    # Define a unit-square polygon as a flat list of (x, y, z) points.
    # The polygon is closed automatically; the last vertex need not repeat
    # the first one.
    square = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
    ]

    # mesh.generate(dim=2) triggers the 2-D triangulator when a polygon has
    # been supplied via option.setString / internal state.  The simplest path
    # is to set the polygon option and then call generate.
    rmsh.option.setString("Mesh.Polygon", str(square))
    rmsh.model.mesh.generate(2)

    # Write out and inspect
    rmsh.write("square_2d.msh")
    print("Wrote 2-D mesh to square_2d.msh")

    rmsh.gui.initialize()
    rmsh.gui.run()

    rmsh.finalize()


# ---------------------------------------------------------------------------
# Example 4: Merge two meshes
# ---------------------------------------------------------------------------
def example_merge(base_path: str, extra_path: str, output_path: str = "merged.msh") -> None:
    """Load a base mesh, merge a second mesh into it, and save the result."""
    rmsh.initialize()

    rmsh.open(base_path)
    rmsh.merge(extra_path)

    rmsh.write(output_path)
    print(f"Merged mesh written to {output_path}")

    rmsh.finalize()


# ---------------------------------------------------------------------------
# Main: run the examples against the bundled test data when available
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    testdata = os.path.join(repo_root, "testdata")

    step_file = os.path.join(testdata, "simple_cube.step")
    msh_file  = os.path.join(testdata, "simple_tetra.msh")  # may not exist yet

    # Run Example 1 if the STEP file is available
    if os.path.exists(step_file):
        print("=== Example 1: STEP → mesh → .msh ===")
        example_step_to_msh(step_file, output_path="simple_cube.msh")
    else:
        print(f"Skipping Example 1: {step_file!r} not found")

    # Run Example 2 if a prebuilt .msh is available
    generated_msh = "simple_cube.msh"
    if os.path.exists(generated_msh):
        print("\n=== Example 2: load .msh → view ===")
        example_load_msh(generated_msh)

    # Run Example 3 unconditionally (no input files needed)
    print("\n=== Example 3: 2-D polygon mesh ===")
    example_2d_polygon_mesh()
