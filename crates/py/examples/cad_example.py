"""rmsh CAD example – build geometry with OCC primitives, mesh, and export.

Usage:
    uv run python examples/cad_example.py
"""

import rmsh

rmsh.initialize()

# --- Create a box 2×1×1 at the origin ---
box_tag = rmsh.model.occ.addBox(0, 0, 0, 2.0, 1.0, 1.0)
print(f"Box   tag = {box_tag}")

# --- Create a sphere at (1, 0.5, 0.5) with radius 0.4 ---
sph_tag = rmsh.model.occ.addSphere(1.0, 0.5, 0.5, 0.4)
print(f"Sphere tag = {sph_tag}")

# --- Create a cylinder along the Z-axis ---
cyl_tag = rmsh.model.occ.addCylinder(0.5, 0.5, -0.5, 0, 0, 2.0, 0.15)
print(f"Cylinder tag = {cyl_tag}")

# --- Synchronize: tessellate all CAD shapes into the mesh ---
rmsh.model.occ.synchronize()
print("OCC synchronize done – surface mesh generated from CAD shapes")

# --- Write STEP file (faceted B-Rep) ---
rmsh.write("cad_example.step")
print("Wrote cad_example.step")

# --- Run meshing algorithm (3-D volume mesh) ---
rmsh.model.mesh.generate(3)
print("Volume mesh generated")

# --- Write the resulting volume mesh ---
rmsh.write("cad_example.msh")
print("Wrote cad_example.msh")

rmsh.finalize()
print("Done.")
