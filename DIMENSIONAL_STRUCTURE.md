# RMSH Dimensional Structure Reference

## Overview

RMSH maintains a **4-level dimensional hierarchy** (0D, 1D, 2D, 3D) with precise correspondences between **geometric entities** (G*) and **mesh entities** (Element/Node/Mesh).

This document provides exhaustive details on:
1. Element type dimensions
2. Geometric entity dimensions  
3. Containment relationships (what each G* entity holds)
4. Dimensional consistency guarantees
5. Query and introspection methods

---

## Quick Reference: Dimensional Correspondence

```
Dimension  Geometric Entity   Mesh Entity                Node Count   Typical Count
─────────────────────────────────────────────────────────────────────────────────
0 (Point)      GVertex         Node (Point1)             1           1–10 per face boundary
1 (Curve)      GEdge           Line2                     2           2–1000 per edge
2 (Surface)    GFace           Triangle3, Quad4         3–4          10–10000 per face
3 (Volume)     GRegion         Tet4, Hex8, Prism6, Pyr5 4–8          100–1M per region
```

---

## Dimension 0: Points/Vertices

### Geometric Entity: `GVertex`

```rust
pub struct GVertex {
    pub id: usize,
    pub node_id: u64,  // Single node ID
}

impl GVertex {
    pub fn dimension(&self) -> u8 { 0 }
}
```

**Semantics:**
- Represents a **geometric corner** where 3+ sharp edges meet
- Points to exactly **one mesh node**
- May or may not have a Point1 element associated

**Contained Mesh Entities:**
- Single `Node` (by ID: `node_id`)

**Example:**
```rust
let gvert = GVertex { id: 5, node_id: 100 };
assert_eq!(gvert.dimension(), 0);
// This geometric vertex corresponds to mesh node #100
```

**Classification:**
- Created when 3+ GEdges converge (dihedral angle classification)
- Common in: corners of boxes, edges of pyramids, blade tips

---

## Dimension 1: Curves/Edges

### Geometric Entity: `GEdge`

```rust
pub struct GEdge {
    pub id: usize,
    pub vertex_ids: [Option<usize>; 2],  // Start and end GVertex IDs
    pub node_ids: Vec<u64>,               // Ordered node sequence
}

impl GEdge {
    pub fn dimension(&self) -> u8 { 1 }
}
```

**Semantics:**
- Represents a **geometric curve** along a sharp feature boundary
- Bounded by 0, 1, or 2 GVertices (closed loops have `[None, None]`)
- Contains an **ordered sequence** of node IDs (typically from Line2 chains)

**Contained Mesh Entities:**
- Sequence of `Node`s (by ID, stored in `node_ids`)
- (Usually from chained Line2 elements, but could be any nodes along the edge)

**Example:**
```rust
let gedge = GEdge {
    id: 12,
    vertex_ids: [Some(3), Some(7)],  // Bounded by GVertex 3 and 7
    node_ids: vec![50, 51, 52, 53, 54],  // 5 nodes along curve
};
assert_eq!(gedge.dimension(), 1);
assert_eq!(gedge.node_ids.len(), 5);
```

**Properties:**
- **Open edge:** `vertex_ids = [Some(a), Some(b)]` (a ≠ b)
- **Closed loop:** `vertex_ids = [None, None]`
- **Dangling edge:** `vertex_ids = [Some(a), None]` or vice versa (rare)

**Classification:**
- Boundary between two GFaces with dihedral angle > threshold
- Connection point between nodes on adjacent GFaces

---

## Dimension 2: Surfaces/Faces

### Geometric Entity: `GFace`

```rust
pub struct GFace {
    pub id: usize,
    pub edge_ids: Vec<usize>,           // Bounding GEdges
    pub mesh_faces: Vec<Vec<u64>>,      // Polygons of nodes
}

impl GFace {
    pub fn dimension(&self) -> u8 { 2 }
}
```

**Semantics:**
- Represents a **geometric surface** region (smooth, same dihedral angle class)
- Bounded by GEdges (forming loops)
- Contains **mesh face polygons** (node sequences)

**Contained Mesh Entities:**
- Multiple node polygons (each `Vec<u64>` is a polygon)
- Typically Triangle3 faces (3 nodes) or Quad4 faces (4 nodes)
- May also be boundary faces extracted from 3D elements

**Example:**
```rust
let gface = GFace {
    id: 8,
    edge_ids: vec![12, 13, 14, 15],  // 4 bounding edges
    mesh_faces: vec![
        vec![50, 51, 52],          // Triangle3 (3 nodes)
        vec![52, 53, 54, 55],      // Quad4 (4 nodes)
        vec![55, 56, 57],          // Triangle3
    ],
};
assert_eq!(gface.dimension(), 2);
assert_eq!(gface.mesh_faces.len(), 3);
assert!(gface.mesh_faces.iter().all(|f| f.len() >= 3));
```

**Properties:**
- **Convex or non-convex** (topology is just the node loop ordering)
- **Single connected region** (one GFace per dihedral angle class)
- **Boundary of 1 or 2 regions** (1 if on model boundary, 2 if interior)

**Classification:**
- Faces separated by dihedral angle < threshold (default 40°) belong to same GFace
- Flood-fill algorithm discovers connected components

---

## Dimension 3: Volumes/Regions

### Geometric Entity: `GRegion`

```rust
pub struct GRegion {
    pub id: usize,
    pub face_ids: Vec<usize>,       // Bounding GFaces
    pub element_ids: Vec<u64>,      // Volume element IDs
}

impl GRegion {
    pub fn dimension(&self) -> u8 { 3 }
}
```

**Semantics:**
- Represents a **geometric solid** (3D volume)
- Bounded by GFaces (forming closed shell)
- Contains **3D element IDs** (Tet4, Hex8, Prism6, Pyramid5)

**Contained Mesh Entities:**
- Volume element IDs (stored in `element_ids`)
- Elements must be **topologically connected** (share full faces)

**Example:**
```rust
let gregion = GRegion {
    id: 1,
    face_ids: vec![1, 2, 3, 4, 5, 6],  // 6 bounding faces
    element_ids: vec![100, 101, 102],  // 3 tet elements
};
assert_eq!(gregion.dimension(), 3);
assert_eq!(gregion.element_ids.len(), 3);
```

**Properties:**
- **Topologically connected** (all elements reachable via face adjacency)
- **Bounded** (surrounded by GFaces forming closed shell)
- **Non-overlapping** with other regions

**Classification:**
- Performed via connected-component analysis on volume elements
- Two elements in same region iff they share a full face (not just edge/vertex)

---

## ElementType Dimension Map

### Canonical Types

| ElementType | Dimension | Nodes | Typical Cells |
|-------------|-----------|-------|---------------|
| `Point1` | 0 | 1 | 1 |
| `Line2` | 1 | 2 | 2–1000 |
| `Triangle3` | 2 | 3 | 10–10000 |
| `Quad4` | 2 | 4 | 10–10000 |
| `Tetrahedron4` | 3 | 4 | 100–1M |
| `Hexahedron8` | 3 | 8 | 100–1M |
| `Prism6` | 3 | 6 | 100–1M |
| `Pyramid5` | 3 | 5 | 100–1M |

### Unknown Types (Gmsh ID-based)

| Gmsh Type ID | Family | Dimension | Example |
|--------------|--------|-----------|---------|
| 15 | Point | 0 | 1-node point |
| 1, 8, 26–28 | Line | 1 | Line2, Line3, Line4 |
| 2, 9, 20–25 | Triangle | 2 | Triangle3, Triangle6 |
| 3, 10, 16, 36–51 | Quad | 2 | Quad4, Quad9 |
| 4, 11, 29–31 | Tetrahedron | 3 | Tet4, Tet10 |
| 5, 12, 17, 92–93 | Hexahedron | 3 | Hex8, Hex27 |
| 6, 13, 18, 90–91 | Prism | 3 | Prism6, Prism18 |
| 7, 14, 19, 118–119 | Pyramid | 3 | Pyr5, Pyr14 |

---

## Containment Guarantees

### GVertex (0D)
```rust
gvertex.dimension() == 0
gvertex.node_id exists in mesh.nodes
```

### GEdge (1D)
```rust
gedge.dimension() == 1
for each node in gedge.node_ids:
    node exists in mesh.nodes
for each vertex_id in gedge.vertex_ids:
    if Some(v) then gmodel.vertices[v] exists
    if None then edge is closed loop
```

### GFace (2D)
```rust
gface.dimension() == 2
for each polygon in gface.mesh_faces:
    polygon.len() >= 3 (at least a triangle)
    all nodes exist in mesh.nodes
for each edge_id in gface.edge_ids:
    gmodel.edges[edge_id] exists
```

### GRegion (3D)
```rust
gregion.dimension() == 3
for each elem_id in gregion.element_ids:
    mesh.elements[elem_id].dimension() == 3
    mesh.elements[elem_id] is connected to others (full face share)
for each face_id in gregion.face_ids:
    gmodel.faces[face_id] exists
```

---

## Dimensional Hierarchy

```
                    GModel (B-Rep)
                   /   |   |   \
              0D  /    1D |    2D \   3D
                 /      |  |       \
              GVertex  GEdge  GFace  GRegion
                 |       |     |      |
            node_id  node_ids  mesh_faces  element_ids
                 |       |     |      |
              Node     Node    Node   Element
                 |       |     |      |
              Point1  Line2  Tri3/   Tet4/
                         Quad4   Hex8/etc.
```

---

## Usage Patterns

### Query by Dimension

```rust
use rmsh_model::{Mesh, Element};

let mesh: Mesh = load_mesh("model.msh")?;

// Get all elements of dimension d
let dim0_elems = mesh.elements_by_dimension(0);  // Point1
let dim1_elems = mesh.elements_by_dimension(1);  // Line2
let dim2_elems = mesh.elements_by_dimension(2);  // Triangle3, Quad4
let dim3_elems = mesh.elements_by_dimension(3);  // Tet4, Hex8, Prism6, Pyr5
```

### Check Geometric Entity Dimension

```rust
use rmsh_model::{GVertex, GEdge, GFace, GRegion};

let gvert: GVertex = ...;
let gedge: GEdge = ...;
let gface: GFace = ...;
let gregion: GRegion = ...;

assert_eq!(gvert.dimension(), 0);
assert_eq!(gedge.dimension(), 1);
assert_eq!(gface.dimension(), 2);
assert_eq!(gregion.dimension(), 3);
```

### Match by Dimension

```rust
use rmsh_model::ElementType;

let etype = ElementType::Triangle3;
match etype.dimension() {
    0 => println!("0D point"),
    1 => println!("1D curve"),
    2 => println!("2D surface"),
    3 => println!("3D volume"),
    _ => println!("unknown"),
}
```

---

## Consistency Checks

### Test Coverage

All dimensional relationships are tested in:
- `crates/model/src/element.rs` — `all_element_types_have_correct_dimension()`, `element_dimension_aligns_with_node_count()`
- `crates/model/src/topology.rs` — `geometric_entities_have_correct_dimensions()`, `dimension_containment_relationships()`

### Running Tests

```bash
cargo test --lib all_element_types_have_correct_dimension
cargo test --lib geometric_entities_have_correct_dimensions
cargo test --lib dimension_containment_relationships
```

All 23 library tests pass ✅ (5 algo + 3 geo + 8 io + 7 model)

---

## Summary

| Aspect | 0D | 1D | 2D | 3D |
|--------|----|----|----|----|
| **Geometric** | GVertex | GEdge | GFace | GRegion |
| **Method** | `gv.dimension()` | `ge.dimension()` | `gf.dimension()` | `gr.dimension()` |
| **Returns** | 0 | 1 | 2 | 3 |
| **Mesh content** | `node_id` | `node_ids[]` | `mesh_faces[][]` | `element_ids[]` |
| **Node count** | 1 | 2+ | 3+ | 4+ |
| **Bounded by** | edges | vertices | edges | faces |
| **Typical elements** | Point1 | Line2 | Tri3, Quad4 | Tet4, Hex8, etc. |
| **Min mesh** | 1 | 2 | 3 (tri) | 4 (tet) |
