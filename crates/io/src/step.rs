use std::collections::HashMap;
use std::fs;
use std::path::Path;

use rmsh_model::{Element, ElementType, Mesh, Node};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum StepError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
    #[error("STEP parse error: {0}")]
    Parse(String),
}

#[derive(Debug, Clone)]
struct Entity {
    name: String,
    args: Vec<String>,
}

pub fn load_step_from_path(path: &Path) -> Result<Mesh, StepError> {
    let data = fs::read(path)?;
    load_step_from_bytes(&data)
}

pub fn load_step_from_bytes(data: &[u8]) -> Result<Mesh, StepError> {
    let text = String::from_utf8_lossy(data);
    parse_step(&text)
}

pub fn parse_step(text: &str) -> Result<Mesh, StepError> {
    let entities = parse_entities(text)?;

    let mut cart_points: HashMap<i64, [f64; 3]> = HashMap::new();
    let mut vertex_points: HashMap<i64, i64> = HashMap::new();
    let mut edge_curves: HashMap<i64, (i64, i64)> = HashMap::new();
    let mut oriented_edges: HashMap<i64, (i64, bool)> = HashMap::new();
    let mut edge_loops: HashMap<i64, Vec<i64>> = HashMap::new();
    let mut face_bounds: HashMap<i64, i64> = HashMap::new();
    let mut advanced_faces: HashMap<i64, Vec<i64>> = HashMap::new();
    let mut closed_shells: HashMap<i64, Vec<i64>> = HashMap::new();
    let mut manifold_solids: Vec<i64> = Vec::new();

    for (&id, ent) in &entities {
        match ent.name.as_str() {
            "CARTESIAN_POINT" => {
                if ent.args.len() >= 2 {
                    if let Some(coords) = parse_cartesian_coords(&ent.args[1]) {
                        cart_points.insert(id, coords);
                    }
                }
            }
            "VERTEX_POINT" => {
                if ent.args.len() >= 2 {
                    if let Some(cp_ref) = parse_ref(&ent.args[1]) {
                        vertex_points.insert(id, cp_ref);
                    }
                }
            }
            "EDGE_CURVE" => {
                if ent.args.len() >= 3 {
                    if let (Some(v1), Some(v2)) = (parse_ref(&ent.args[1]), parse_ref(&ent.args[2])) {
                        edge_curves.insert(id, (v1, v2));
                    }
                }
            }
            "ORIENTED_EDGE" => {
                if ent.args.len() >= 5 {
                    if let Some(ec_ref) = parse_ref(&ent.args[3]) {
                        let same_sense = parse_step_bool(&ent.args[4]).unwrap_or(true);
                        oriented_edges.insert(id, (ec_ref, same_sense));
                    }
                }
            }
            "EDGE_LOOP" => {
                if ent.args.len() >= 2 {
                    let refs = parse_ref_list(&ent.args[1]);
                    edge_loops.insert(id, refs);
                }
            }
            "FACE_OUTER_BOUND" | "FACE_BOUND" => {
                if ent.args.len() >= 2 {
                    if let Some(loop_ref) = parse_ref(&ent.args[1]) {
                        face_bounds.insert(id, loop_ref);
                    }
                }
            }
            "ADVANCED_FACE" => {
                if ent.args.len() >= 2 {
                    advanced_faces.insert(id, parse_ref_list(&ent.args[1]));
                }
            }
            "CLOSED_SHELL" => {
                if ent.args.len() >= 2 {
                    closed_shells.insert(id, parse_ref_list(&ent.args[1]));
                }
            }
            "MANIFOLD_SOLID_BREP" => {
                if ent.args.len() >= 2 {
                    if let Some(shell_ref) = parse_ref(&ent.args[1]) {
                        manifold_solids.push(shell_ref);
                    }
                }
            }
            _ => {}
        }
    }

    let mut target_face_ids: Vec<i64> = Vec::new();
    if let Some(shell_ref) = manifold_solids.first() {
        if let Some(face_ids) = closed_shells.get(shell_ref) {
            target_face_ids.extend(face_ids.iter().copied());
        }
    }
    if target_face_ids.is_empty() {
        if let Some((_sid, face_ids)) = closed_shells.iter().next() {
            target_face_ids.extend(face_ids.iter().copied());
        }
    }
    if target_face_ids.is_empty() {
        target_face_ids.extend(advanced_faces.keys().copied());
        target_face_ids.sort_unstable();
    }

    let mut mesh = Mesh::new();
    let mut cp_to_node: HashMap<i64, u64> = HashMap::new();
    let mut next_node_id: u64 = 1;
    let mut next_elem_id: u64 = 1;

    for face_id in target_face_ids {
        let Some(bounds) = advanced_faces.get(&face_id) else {
            continue;
        };
        let Some(bound_ref) = bounds.first() else {
            continue;
        };
        let Some(loop_ref) = face_bounds.get(bound_ref) else {
            continue;
        };
        let Some(oe_refs) = edge_loops.get(loop_ref) else {
            continue;
        };

        let mut vrefs: Vec<i64> = Vec::new();
        for oe_ref in oe_refs {
            let Some((ec_ref, same_sense)) = oriented_edges.get(oe_ref).copied() else {
                continue;
            };
            let Some((v1, v2)) = edge_curves.get(&ec_ref).copied() else {
                continue;
            };
            let (a, b) = if same_sense { (v1, v2) } else { (v2, v1) };

            if vrefs.is_empty() {
                vrefs.push(a);
                vrefs.push(b);
            } else {
                let last = *vrefs.last().unwrap_or(&a);
                if last == a {
                    vrefs.push(b);
                } else if last == b {
                    vrefs.push(a);
                } else if vrefs.first().copied() == Some(b) {
                    vrefs.push(a);
                } else if vrefs.first().copied() == Some(a) {
                    vrefs.push(b);
                } else {
                    // Disconnected edge in loop ordering; skip it in minimal parser.
                    continue;
                }
            }
        }

        if vrefs.len() >= 2 && vrefs.first() == vrefs.last() {
            vrefs.pop();
        }
        // Remove consecutive duplicates
        vrefs.dedup();

        let mut face_node_ids: Vec<u64> = Vec::new();
        for vref in vrefs {
            let Some(cp_ref) = vertex_points.get(&vref).copied() else {
                continue;
            };
            let Some(coords) = cart_points.get(&cp_ref).copied() else {
                continue;
            };

            let nid = if let Some(id) = cp_to_node.get(&cp_ref).copied() {
                id
            } else {
                let id = next_node_id;
                next_node_id += 1;
                mesh.add_node(Node::new(id, coords[0], coords[1], coords[2]));
                cp_to_node.insert(cp_ref, id);
                id
            };
            if face_node_ids.last().copied() != Some(nid) {
                face_node_ids.push(nid);
            }
        }

        if face_node_ids.len() < 3 {
            continue;
        }

        if face_node_ids.len() == 3 {
            mesh.add_element(Element::new(next_elem_id, ElementType::Triangle3, face_node_ids));
            next_elem_id += 1;
        } else if face_node_ids.len() == 4 {
            mesh.add_element(Element::new(next_elem_id, ElementType::Quad4, face_node_ids));
            next_elem_id += 1;
        } else {
            // Simple fan triangulation for polygonal faces.
            let root = face_node_ids[0];
            for i in 1..(face_node_ids.len() - 1) {
                let tri = vec![root, face_node_ids[i], face_node_ids[i + 1]];
                mesh.add_element(Element::new(next_elem_id, ElementType::Triangle3, tri));
                next_elem_id += 1;
            }
        }
    }

    if mesh.node_count() == 0 || mesh.element_count() == 0 {
        return Err(StepError::Parse(
            "No polygonal faces found in STEP data (expected simple faceted B-Rep)".to_string(),
        ));
    }

    Ok(mesh)
}

fn parse_entities(text: &str) -> Result<HashMap<i64, Entity>, StepError> {
    let mut entities: HashMap<i64, Entity> = HashMap::new();
    let mut stmt = String::new();

    for ch in text.chars() {
        stmt.push(ch);
        if ch == ';' {
            let raw = stmt.trim();
            if let Some((id, ent)) = parse_entity_statement(raw)? {
                entities.insert(id, ent);
            }
            stmt.clear();
        }
    }

    Ok(entities)
}

fn parse_entity_statement(raw: &str) -> Result<Option<(i64, Entity)>, StepError> {
    let statement = raw.trim_end_matches(';').trim();
    if !statement.starts_with('#') {
        return Ok(None);
    }

    let Some(eq_pos) = statement.find('=') else {
        return Err(StepError::Parse(format!("Malformed entity statement: {statement}")));
    };

    let id_str = statement[1..eq_pos].trim();
    let id: i64 = id_str
        .parse()
        .map_err(|_| StepError::Parse(format!("Invalid entity id: {id_str}")))?;

    let rhs = statement[(eq_pos + 1)..].trim();
    let Some(lp) = rhs.find('(') else {
        return Ok(None);
    };
    if !rhs.ends_with(')') {
        return Ok(None);
    }

    let name = rhs[..lp].trim().to_ascii_uppercase();
    let args_str = &rhs[(lp + 1)..(rhs.len() - 1)];
    let args = split_top_level_args(args_str);

    Ok(Some((id, Entity { name, args })))
}

fn split_top_level_args(s: &str) -> Vec<String> {
    let mut args = Vec::new();
    let mut start = 0usize;
    let mut depth = 0i32;
    let mut in_string = false;
    let chars: Vec<char> = s.chars().collect();

    let mut i = 0usize;
    while i < chars.len() {
        let c = chars[i];
        if c == '\'' {
            // STEP string escaping uses doubled single quote.
            if in_string && i + 1 < chars.len() && chars[i + 1] == '\'' {
                i += 1;
            } else {
                in_string = !in_string;
            }
        } else if !in_string {
            match c {
                '(' => depth += 1,
                ')' => depth -= 1,
                ',' if depth == 0 => {
                    args.push(s[start..i].trim().to_string());
                    start = i + 1;
                }
                _ => {}
            }
        }
        i += 1;
    }

    if start < s.len() {
        args.push(s[start..].trim().to_string());
    }

    args
}

fn parse_ref(token: &str) -> Option<i64> {
    let t = token.trim();
    if let Some(rest) = t.strip_prefix('#') {
        rest.parse::<i64>().ok()
    } else {
        None
    }
}

fn parse_step_bool(token: &str) -> Option<bool> {
    match token.trim().to_ascii_uppercase().as_str() {
        ".T." => Some(true),
        ".F." => Some(false),
        _ => None,
    }
}

fn parse_ref_list(token: &str) -> Vec<i64> {
    let t = token.trim();
    let inner = t.strip_prefix('(').and_then(|v| v.strip_suffix(')')).unwrap_or(t);
    split_top_level_args(inner)
        .into_iter()
        .filter_map(|x| parse_ref(&x))
        .collect()
}

fn parse_cartesian_coords(token: &str) -> Option<[f64; 3]> {
    let t = token.trim();
    let inner = t.strip_prefix('(').and_then(|v| v.strip_suffix(')')).unwrap_or(t);
    let coords: Vec<f64> = split_top_level_args(inner)
        .into_iter()
        .filter_map(|x| x.parse::<f64>().ok())
        .collect();

    if coords.len() >= 3 {
        Some([coords[0], coords[1], coords[2]])
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::{load_step_from_path, parse_step};
    use std::path::PathBuf;

    #[test]
    fn parse_simple_tetra_faceted_brep() {
        let step = r#"
ISO-10303-21;
HEADER;
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
#21=EDGE_CURVE('',#11,#12,#999,.T.);
#22=EDGE_CURVE('',#12,#13,#999,.T.);
#23=EDGE_CURVE('',#13,#11,#999,.T.);
#24=EDGE_CURVE('',#11,#14,#999,.T.);
#25=EDGE_CURVE('',#12,#14,#999,.T.);
#26=EDGE_CURVE('',#13,#14,#999,.T.);
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
#71=ADVANCED_FACE('',(#61),#998,.T.);
#72=ADVANCED_FACE('',(#62),#998,.T.);
#73=ADVANCED_FACE('',(#63),#998,.T.);
#74=ADVANCED_FACE('',(#64),#998,.T.);
#81=CLOSED_SHELL('',(#71,#72,#73,#74));
#82=MANIFOLD_SOLID_BREP('',#81);
ENDSEC;
END-ISO-10303-21;
"#;

        let mesh = parse_step(step).expect("STEP should parse");
        assert_eq!(mesh.node_count(), 4);
        assert_eq!(mesh.element_count(), 4);
    }

    #[test]
    fn parse_generated_step_test_file() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join("testdata")
            .join("simple_tetra.step");

        let mesh = load_step_from_path(&path).expect("generated STEP file should parse");
        assert_eq!(mesh.node_count(), 4);
        assert_eq!(mesh.element_count(), 4);
    }

    #[test]
    fn parse_generated_step_cube_file() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join("testdata")
            .join("simple_cube.step");

        let mesh = load_step_from_path(&path).expect("generated cube STEP file should parse");
        assert_eq!(mesh.node_count(), 8);
        assert_eq!(mesh.element_count(), 6);
        assert!(mesh.elements.iter().all(|e| e.node_ids.len() == 4));
    }

    #[test]
    fn parse_my_cube_step_file() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join("testdata")
            .join("my_cube.step");

        let mesh = load_step_from_path(&path).expect("my_cube.step should parse");
        // 8 unique corner vertices, 6 quad faces
        assert_eq!(mesh.node_count(), 8, "expected 8 corner nodes");
        assert_eq!(mesh.element_count(), 6, "expected 6 quad faces");
        assert!(mesh.elements.iter().all(|e| e.node_ids.len() == 4));
    }
}
