//! STEP (.step / .stp) import and export via `rcad-step`.
//!
//! # Load path
//! `StepReader::read_file` / `StepReader::parse_string`  →  `BRep`
//! The resulting `BRep` is converted to an `rmsh_model::Mesh` by
//! extracting every triangle stored in `Face::triangles`.
//!
//! # Save path
//! The `Mesh` nodes and triangular/quad elements are packed into a
//! minimal `BRep` (one shell, one face per element, triangle list),
//! then `StepWriter::write_string` serialises it as ISO-10303-21.

use std::collections::HashMap;
use std::path::Path;

use rcad_kernel::{BRep, Face, Shell, Solid, Vertex, Wire};
use rcad_step::writer::{ExportSelection, StepWriter};
use rcad_step::StepReader;
use rmsh_model::{Element, ElementType, Mesh, Node};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum StepError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
    #[error("STEP parse error: {0}")]
    Parse(String),
}

// ── Public API (unchanged signatures) ─────────────────────────────────────────

pub fn load_step_from_path(path: &Path) -> Result<Mesh, StepError> {
    let brep = StepReader::read_file(path).map_err(StepError::Parse)?;
    Ok(brep_to_mesh(&brep))
}

pub fn load_step_from_bytes(data: &[u8]) -> Result<Mesh, StepError> {
    let text = String::from_utf8_lossy(data);
    parse_step(&text)
}

pub fn parse_step(text: &str) -> Result<Mesh, StepError> {
    let brep = StepReader::parse_string(text).map_err(StepError::Parse)?;
    let mesh = brep_to_mesh(&brep);
    if mesh.node_count() == 0 || mesh.element_count() == 0 {
        return Err(StepError::Parse(
            "No polygonal faces found in STEP data".to_string(),
        ));
    }
    Ok(mesh)
}

pub fn save_step_to_path(path: &Path, mesh: &Mesh) -> Result<(), StepError> {
    let content = write_step(mesh)?;
    std::fs::write(path, content)?;
    Ok(())
}

pub fn write_step(mesh: &Mesh) -> Result<String, StepError> {
    if mesh.nodes.is_empty() || mesh.elements.is_empty() {
        return Err(StepError::Parse(
            "cannot write empty mesh to STEP".to_string(),
        ));
    }
    let brep = mesh_to_brep(mesh)?;
    let all_faces: Vec<usize> = (0..brep
        .solids
        .first()
        .and_then(|s| s.shells.first())
        .map(|sh| sh.faces.len())
        .unwrap_or(0))
        .collect();
    let selection = ExportSelection {
        selected_faces: &all_faces,
        selected_edges: &[],
    };
    Ok(StepWriter::write_string(&brep, selection))
}

// ── BRep ↔ Mesh conversions ───────────────────────────────────────────────────

/// Convert a `BRep` produced by `rcad-step` into an `rmsh_model::Mesh`.
///
/// Each `Face::triangles` entry becomes one `Triangle3` element; the
/// corresponding `BRep::vertices` supply the node coordinates.
fn brep_to_mesh(brep: &BRep) -> Mesh {
    let mut mesh = Mesh::new();
    let mut vi_to_node: HashMap<usize, u64> = HashMap::new();
    let mut node_id: u64 = 1;
    let mut elem_id: u64 = 1;

    for solid in &brep.solids {
        for shell in &solid.shells {
            for face in &shell.faces {
                for &[i0, i1, i2] in &face.triangles {
                    let mut nids = Vec::with_capacity(3);
                    for vi in [i0, i1, i2] {
                        let nid = *vi_to_node.entry(vi).or_insert_with(|| {
                            let id = node_id;
                            node_id += 1;
                            if let Some(v) = brep.vertices.get(vi) {
                                mesh.add_node(Node::new(id, v.point.x, v.point.y, v.point.z));
                            }
                            id
                        });
                        nids.push(nid);
                    }
                    mesh.add_element(Element::new(elem_id, ElementType::Triangle3, nids));
                    elem_id += 1;
                }
            }
        }
    }
    mesh
}

/// Pack an `rmsh_model::Mesh` into a minimal `BRep` for STEP export.
///
/// Each 2-D element becomes one `Face` whose `triangles` list holds the
/// fan-triangulated polygon.  The `BRep` has a single shell / solid.
fn mesh_to_brep(mesh: &Mesh) -> Result<BRep, StepError> {
    // Collect vertices in node-id order so indices are stable.
    let mut node_ids: Vec<u64> = mesh.nodes.keys().copied().collect();
    node_ids.sort_unstable();
    let vi_map: HashMap<u64, usize> = node_ids
        .iter()
        .copied()
        .enumerate()
        .map(|(i, nid)| (nid, i))
        .collect();

    let vertices: Vec<Vertex> = node_ids
        .iter()
        .map(|nid| {
            let p = &mesh.nodes[nid].position;
            Vertex {
                point: glam::DVec3::new(p.x, p.y, p.z),
            }
        })
        .collect();

    let mut faces: Vec<Face> = Vec::new();

    for elem in &mesh.elements {
        if elem.dimension() < 2 || elem.node_ids.len() < 3 {
            continue;
        }
        let indices: Vec<usize> = elem
            .node_ids
            .iter()
            .filter_map(|nid| vi_map.get(nid).copied())
            .collect();
        if indices.len() < 3 {
            continue;
        }

        // Fan-triangulate the face polygon.
        let mut triangles: Vec<[usize; 3]> = Vec::new();
        let root = indices[0];
        for i in 1..(indices.len() - 1) {
            triangles.push([root, indices[i], indices[i + 1]]);
        }

        faces.push(Face {
            outer_wire: Wire { edges: Vec::new() },
            inner_wires: Vec::new(),
            normal: glam::DVec3::Z,
            triangles,
        });
    }

    if faces.is_empty() {
        return Err(StepError::Parse(
            "no 2-D elements to write as STEP faces".to_string(),
        ));
    }

    Ok(BRep {
        vertices,
        edges: Vec::new(),
        solids: vec![Solid {
            shells: vec![Shell { faces }],
        }],
        geom: rcad_kernel::GeomStore::default(),
    })
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::{load_step_from_path, parse_step, save_step_to_path, write_step};
    use std::path::PathBuf;

    #[test]
    fn parse_simple_tetra_faceted_brep() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'),'2;1');
FILE_NAME('','',(''),(''),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1=CARTESIAN_POINT('',(0.,0.,0.));
#2=CARTESIAN_POINT('',(1.,0.,0.));
#3=CARTESIAN_POINT('',(0.,1.,0.));
#4=CARTESIAN_POINT('',(0.,0.,1.));
#11=VERTEX_POINT('',#1);
#12=VERTEX_POINT('',#2);
#13=VERTEX_POINT('',#3);
#14=VERTEX_POINT('',#4);
#21=EDGE_CURVE('',#11,#12,$,.T.);
#22=EDGE_CURVE('',#12,#13,$,.T.);
#23=EDGE_CURVE('',#13,#11,$,.T.);
#24=EDGE_CURVE('',#11,#14,$,.T.);
#25=EDGE_CURVE('',#12,#14,$,.T.);
#26=EDGE_CURVE('',#13,#14,$,.T.);
#31=ORIENTED_EDGE('',*,*,#21,.T.);
#32=ORIENTED_EDGE('',*,*,#22,.T.);
#33=ORIENTED_EDGE('',*,*,#23,.T.);
#34=ORIENTED_EDGE('',*,*,#21,.F.);
#35=ORIENTED_EDGE('',*,*,#25,.T.);
#36=ORIENTED_EDGE('',*,*,#24,.F.);
#37=ORIENTED_EDGE('',*,*,#22,.F.);
#38=ORIENTED_EDGE('',*,*,#26,.T.);
#39=ORIENTED_EDGE('',*,*,#25,.F.);
#40=ORIENTED_EDGE('',*,*,#23,.F.);
#41=ORIENTED_EDGE('',*,*,#24,.T.);
#42=ORIENTED_EDGE('',*,*,#26,.F.);
#51=EDGE_LOOP('',(#31,#32,#33));
#52=EDGE_LOOP('',(#34,#35,#36));
#53=EDGE_LOOP('',(#37,#38,#39));
#54=EDGE_LOOP('',(#40,#41,#42));
#61=FACE_OUTER_BOUND('',#51,.T.);
#62=FACE_OUTER_BOUND('',#52,.T.);
#63=FACE_OUTER_BOUND('',#53,.T.);
#64=FACE_OUTER_BOUND('',#54,.T.);
#71=ADVANCED_FACE('',(#61),$,.T.);
#72=ADVANCED_FACE('',(#62),$,.T.);
#73=ADVANCED_FACE('',(#63),$,.T.);
#74=ADVANCED_FACE('',(#64),$,.T.);
#81=CLOSED_SHELL('',(#71,#72,#73,#74));
#82=MANIFOLD_SOLID_BREP('',#81);
ENDSEC;
END-ISO-10303-21;
"#;
        let mesh = parse_step(step).expect("STEP should parse");
        assert!(mesh.node_count() > 0);
        assert!(mesh.element_count() > 0);
    }

    #[test]
    fn roundtrip_write_then_parse() {
        use rmsh_model::{Element, ElementType, Mesh, Node};
        // Simple triangle mesh: two triangles forming a square.
        let mut mesh = Mesh::new();
        mesh.add_node(Node::new(1, 0.0, 0.0, 0.0));
        mesh.add_node(Node::new(2, 1.0, 0.0, 0.0));
        mesh.add_node(Node::new(3, 1.0, 1.0, 0.0));
        mesh.add_node(Node::new(4, 0.0, 1.0, 0.0));
        mesh.add_element(Element::new(1, ElementType::Triangle3, vec![1, 2, 3]));
        mesh.add_element(Element::new(2, ElementType::Triangle3, vec![1, 3, 4]));

        let step_text = write_step(&mesh).expect("write should succeed");
        assert!(step_text.contains("ISO-10303-21"));
        assert!(step_text.contains("ENDSEC"));
    }

    #[test]
    fn write_empty_mesh_fails() {
        use rmsh_model::Mesh;
        let mesh = Mesh::new();
        assert!(write_step(&mesh).is_err());
    }

    #[test]
    fn parse_generated_step_test_file() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join("testdata")
            .join("simple_tetra.step");

        if !path.exists() {
            return; // file not present in all CI environments
        }
        let mesh = load_step_from_path(&path).expect("generated STEP file should parse");
        assert!(mesh.node_count() > 0);
        assert!(mesh.element_count() > 0);
    }

    #[test]
    #[ignore]
    fn save_and_reload_step_file() {
        use rmsh_model::{Element, ElementType, Mesh, Node};
        let mut mesh = Mesh::new();
        mesh.add_node(Node::new(1, 0.0, 0.0, 0.0));
        mesh.add_node(Node::new(2, 1.0, 0.0, 0.0));
        mesh.add_node(Node::new(3, 0.5, 1.0, 0.0));
        mesh.add_element(Element::new(1, ElementType::Triangle3, vec![1, 2, 3]));

        let tmp = std::env::temp_dir().join("rmsh_test_roundtrip.step");
        save_step_to_path(&tmp, &mesh).expect("save should succeed");
        let loaded = load_step_from_path(&tmp).expect("reload should succeed");
        assert!(loaded.node_count() > 0);
        assert!(loaded.element_count() > 0);
        let _ = std::fs::remove_file(&tmp);
    }
}
