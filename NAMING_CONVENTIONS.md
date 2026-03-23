# RMSH Naming Conventions (Gmsh-Aligned)

This document explains the entity naming scheme used in RMSH, which follows Gmsh's conventions to clearly distinguish **geometric entities** from **finite element mesh entities**.

## Overview

RMSH code operates at two conceptual levels:

1. **Geometric Model (G\*)** — B-Rep topology extracted from CAD files (STEP) or inferred from mesh via dihedral-angle classification
2. **Mesh (M\*, future)** — Finite element discretization (nodes, elements)

This distinction helps prevent confusion when the same mesh contains both geometric and discretized representations.

---

## Type Naming Conventions

### Geometric Entities (G Prefix)

Geometric entities represent the high-level CAD/topological structure:

| Type | Purpose | Details |
|------|---------|---------|
| `GVertex` | Geometric point/corner | Where 3+ edges meet; contains reference to mesh node ID |
| `GEdge` | Geometric curve | Sharp feature boundary; contains ordered sequence of mesh node IDs |
| `GFace` | Geometric surface | Smooth region bounded by edges; contains mesh face polygons |
| `GRegion` | Geometric solid volume | 3D domain; contains mesh element IDs and bounding face IDs |
| `GModel` | Geometric model container | B-Rep topology with all GVertices, GEdges, GFaces, GRegions |
| `GSelection` | Selection enum | `Region(id)`, `Face(id)`, `Edge(id)`, `Vertex(id)` |

### Mesh Entities (Future M Prefix)

Currently (work in progress), mesh entities lack the M prefix but will be renamed for consistency:

| Type | Current Name | Planned Name | Purpose |
|------|--------------|--------------|---------|
| Mesh node | `Node` | `MVertex` (future) | Point in finite element discretization |
| Mesh element | `Element` | `MElement` (future) | Finite element (Triangle3, Quad4, Tet4, etc.) |
| Mesh container | `Mesh` | `MMesh` (optional) | Container of nodes and elements |

---

## Usage Examples

### Creating a Geometric Model

```rust
use rmsh_model::GModel;
use rmsh_geo::classify;

let mesh = load_mesh_from_file("model.msh")?;
let gmodel: GModel = classify::classify(&mesh, 40.0);  // 40° dihedral angle threshold

for gregion in &gmodel.regions {
    println!("Region {}: {} elements", gregion.id, gregion.element_ids.len());
}

for gface in &gmodel.faces {
    println!("Face {}: {} bounding edges", gface.id, gface.edge_ids.len());
}
```

### Selecting Geometric Entities

```rust
use rmsh_model::GSelection;
use rmsh_geo::extract;

let selection = GSelection::Face(3);  // Select face with ID 3
let (surface_data, _) = extract::extract_highlight(&mesh, &gmodel, &selection);
```

### Backward Compatibility

For legacy code, type aliases are provided:

```rust
// Old names (deprecated)
use rmsh_model::{Topology, TopoVertex, TopoEdge, TopoFace, TopoVolume, TopoSelection};

// These are type aliases to the new names:
// Topology ≡ GModel
// TopoVertex ≡ GVertex
// TopoEdge ≡ GEdge
// TopoFace ≡ GFace
// TopoVolume ≡ GRegion
// TopoSelection ≡ GSelection
```

---

## Classification Algorithm

The `GModel` is constructed via **dihedral-angle based classification** (inspired by Gmsh):

1. **Extract boundary faces** from volume elements (or use 2D elements directly)
2. **Build face adjacency graph** via shared edges
3. **Compute dihedral angle** between adjacent faces (normal-to-normal angle)
4. **Flood-fill faces** into GFaces: faces separated by angle < threshold belong to same GFace
5. **Identify GEdges** as boundaries between different GFaces
6. **Identify GVertices** as points where 3+ GEdges meet
7. **Group elements into GRegions** by connected volume element components

**Default threshold:** 40 degrees (tunable via `GModel::angle_threshold_deg`)

---

## File Organization

- **model/src/topology.rs** — G* type definitions and aliases
- **geo/src/classify.rs** — Algorithm to compute GModel from Mesh
- **geo/src/extract.rs** — Extract GModel subsets for rendering
- **viewer/src/app.rs** — UI interaction with GModel and GSelection
- **io/src/msh.rs, step.rs** — Loaders producing Mesh (not GModel)

---

## Rationale

This naming convention:
- **Clarifies intent** when reading code (G* = geometric, M* = mesh)
- **Reduces confusion** in functions working with both representations
- **Enables future extensions** (e.g., multi-scale analysis mixing geometry + mesh)
- **Aligns with industry standards** (Gmsh, Salome, ParaView all use similar prefixes)

---

## Migration Guide (Future: Node→MVertex, Element→MElement)

When mesh entities are renamed:

**Before:**
```rust
let mesh: Mesh = load_msh("file.msh")?;
for node in &mesh.nodes.values() {
    println!("Node {} at {}", node.id, node.position);
}
```

**After:**
```rust
let mesh: Mesh = load_msh("file.msh")?;
for mvertex in &mesh.nodes.values() {  // or mesh.mvertices()
    println!("MVertex {} at {}", mvertex.id, mvertex.position);
}
```

The `Mesh` container likely stays as-is for brevity.

---

## Dimensional Correspondence (0D–3D)

RMSH maintains a **dimensional hierarchy** where geometric entities contain mesh entities of the same dimension:

### Dimension 0 (Point)

| From | Geometric Entity | Mesh Entity | Size | Example |
|------|------------------|-------------|------|---------|
| STEP/CAD | `GVertex` | `Node` | 1 node | point/corner vertex |
| Criteria | 3+ edges meet | Single point | — | ID=5 @ (1.0, 2.0, 3.0) |
| Method | `GVertex::dimension() → 0` | `Node` (implicit) | — | — |

**Containment:** `GVertex` → single `node_id`

---

### Dimension 1 (Curve/Edge)

| From | Geometric Entity | Mesh Entity | Size | Example |
|------|------------------|-------------|------|---------|
| CAD reconstruction | `GEdge` | `Line2` elements | 2+ nodes | sharp boundary |
| Criteria | boundary between 2 faces | edge element | — | — |
| Mesh path | ordered `node_ids` | `ElementType::Line2` | 2 nodes | chain of line elements |
| Method | `GEdge::dimension() → 1` | `Element::dimension() → 1` | — | — |

**Containment:** `GEdge` → ordered `node_ids` (typically from Line2 chains) + endpoints from `GVertex`

Example: `GEdge { id: 2, vertex_ids: [Some(1), Some(3)], node_ids: [10, 11, 12, 13] }`

---

### Dimension 2 (Surface/Face)

| From | Geometric Entity | Mesh Entity | Size | Example |
|------|------------------|-------------|------|---------|
| CAD/Dihedral flood-fill | `GFace` | Triangle3, Quad4 | 3–4 nodes each | smooth surface |
| Criteria | adjacent faces, angle < threshold | face element | — | — |
| Mesh path | `mesh_faces` (node polygons) | `ElementType::Triangle3`, `Quad4` | 3 or 4 | triangle or quad |
| Method | `GFace::dimension() → 2` | `Element::dimension() → 2` | — | — |

**Containment:** `GFace` → `mesh_faces = Vec<Vec<u64>>` (each Vec is a polygon of node IDs)

Example: `GFace { id: 4, edge_ids: [1,2,3], mesh_faces: [[10,11,12], [12,13,14,15]] }`

---

### Dimension 3 (Volume/Region)

| From | Geometric Entity | Mesh Entity | Size | Example |
|------|------------------|-------------|------|---------|
| Volume reconstruction | `GRegion` | Tet4, Hex8, Prism6, Pyramid5 | 4, 8, 6, 5 nodes | solid domain |
| Criteria | connected volume elements | volume element | — | — |
| Mesh path | `element_ids` | `ElementType::Tetrahedron4`, `Hexahedron8`, `Prism6`, `Pyramid5` | — | 3D element |
| Method | `GRegion::dimension() → 3` | `Element::dimension() → 3` | — | — |

**Containment:** `GRegion` → `element_ids` (all 3D elements in connected domain)

Example: `GRegion { id: 1, face_ids: [1,2,3,4,5,6], element_ids: [100,101,102] }`

---

## Element Type Reference

### All Supported Element Types (with dimensions)

```rust
// 0-Dimensional
Point1          // 1 node

// 1-Dimensional
Line2           // 2 nodes

// 2-Dimensional
Triangle3       // 3 nodes
Quad4           // 4 nodes

// 3-Dimensional
Tetrahedron4    // 4 nodes
Hexahedron8     // 8 nodes (cube)
Prism6          // 6 nodes (wedge / triangular prism)
Pyramid5        // 5 nodes (quad base + apex)

// Special
Unknown(i32)    // Dimension inferred from Gmsh type ID
```

### Dimension Lookup

```rust
use rmsh_model::ElementType;

let elem = ElementType::Triangle3;
assert_eq!(elem.dimension(), 2);  // Surface element

// Get by Gmsh type ID
let elem2 = ElementType::from_gmsh_type_id(2);  // Triangle3
assert_eq!(elem2.dimension(), 2);
```

---

## Mesh/Geometry Introspection

### Query mesh by dimension

```rust
use rmsh_model::{Mesh, Element};

let mesh: Mesh = load_mesh_from_file("model.msh")?;

// Get all 3D volume elements
let volume_elems: Vec<&Element> = mesh.elements_by_dimension(3);
assert!(volume_elems.iter().all(|e| e.dimension() == 3));

// Get all 2D surface elements
let surf_elems: Vec<&Element> = mesh.elements_by_dimension(2);

// Get all 1D edge elements
let edge_elems: Vec<&Element> = mesh.elements_by_dimension(1);

// Get all 0D point elements
let point_elems: Vec<&Element> = mesh.elements_by_dimension(0);
```

### Check geometric entity dimension

```rust
use rmsh_model::{GVertex, GEdge, GFace, GRegion};

let gvert = GVertex { id: 1, node_id: 5 };
assert_eq!(gvert.dimension(), 0);

let gedge = GEdge { id: 2, vertex_ids: [Some(1), Some(3)], node_ids: vec![10, 11, 12] };
assert_eq!(gedge.dimension(), 1);

let gface = GFace { id: 4, edge_ids: vec![1, 2], mesh_faces: vec![vec![10, 11, 12]] };
assert_eq!(gface.dimension(), 2);

let gregion = GRegion { id: 1, face_ids: vec![1, 2, 3], element_ids: vec![100, 101] };
assert_eq!(gregion.dimension(), 3);
```

---

## Summary Table

| Dimension | Geometric | Mesh Entities | Typical Use | Gmsh Equivalent |
|-----------|-----------|---------------|-------------|-----------------|
| **0** | `GVertex` | `Node`, `Point1` | Corners, vertices | 0-Point |
| **1** | `GEdge` | `Line2` | Curves, sharp edges | 1-Curve |
| **2** | `GFace` | `Triangle3`, `Quad4` | Surfaces, faces | 2-Surface |
| **3** | `GRegion` | `Tetrahedron4`, `Hexahedron8`, `Prism6`, `Pyramid5` | Solids, volumes | 3-Volume |

