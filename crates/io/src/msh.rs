use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufRead, BufReader, Cursor, Write};
use std::path::Path;

use rmsh_model::{Element, ElementType, Mesh, Node};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum MshError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
    #[error("Parse error at line {line}: {message}")]
    Parse { line: usize, message: String },
    #[error("Unsupported MSH format version: {0}")]
    UnsupportedVersion(String),
    #[error("Unsupported element type for MSH write: {0:?}")]
    UnsupportedElementType(ElementType),
    #[error("Element references missing node ID: {0}")]
    MissingNode(u64),
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum MshVersion {
    V2,
    V4,
}

pub fn load_msh_from_path(path: &Path) -> Result<Mesh, MshError> {
    let file = File::open(path)?;
    let reader = BufReader::new(file);
    parse_msh(reader)
}

pub fn load_msh_from_bytes(data: &[u8]) -> Result<Mesh, MshError> {
    parse_msh(Cursor::new(data))
}

pub fn save_msh_v2_to_path(path: &Path, mesh: &Mesh) -> Result<(), MshError> {
    let mut file = File::create(path)?;
    write_msh_v2(&mut file, mesh)
}

pub fn save_msh_v4_to_path(path: &Path, mesh: &Mesh) -> Result<(), MshError> {
    let mut file = File::create(path)?;
    write_msh_v4(&mut file, mesh)
}

pub fn write_msh_v2<W: Write>(writer: &mut W, mesh: &Mesh) -> Result<(), MshError> {
    validate_mesh(mesh)?;

    let nodes = sorted_nodes(mesh);
    let elements = sorted_elements(mesh);

    writeln!(writer, "$MeshFormat")?;
    writeln!(writer, "2.2 0 8")?;
    writeln!(writer, "$EndMeshFormat")?;
    write_physical_names(writer, mesh)?;

    writeln!(writer, "$Nodes")?;
    writeln!(writer, "{}", nodes.len())?;
    for node in nodes {
        writeln!(
            writer,
            "{} {} {} {}",
            node.id, node.position.x, node.position.y, node.position.z
        )?;
    }
    writeln!(writer, "$EndNodes")?;

    writeln!(writer, "$Elements")?;
    writeln!(writer, "{}", elements.len())?;
    for element in elements {
        let etype = gmsh_type_id(element.etype)?;
        match element.physical_tag {
            Some(physical_tag) => {
                write!(writer, "{} {} 2 {} 0", element.id, etype, physical_tag)?;
            }
            None => {
                write!(writer, "{} {} 0", element.id, etype)?;
            }
        }
        for node_id in &element.node_ids {
            write!(writer, " {}", node_id)?;
        }
        writeln!(writer)?;
    }
    writeln!(writer, "$EndElements")?;

    Ok(())
}

pub fn write_msh_v4<W: Write>(writer: &mut W, mesh: &Mesh) -> Result<(), MshError> {
    validate_mesh(mesh)?;

    let nodes = sorted_nodes(mesh);
    let elements = sorted_elements(mesh);
    let min_node_tag = nodes.first().map(|node| node.id).unwrap_or(0);
    let max_node_tag = nodes.last().map(|node| node.id).unwrap_or(0);
    let min_element_tag = elements.first().map(|element| element.id).unwrap_or(0);
    let max_element_tag = elements.last().map(|element| element.id).unwrap_or(0);
    let entity_dim = elements
        .iter()
        .map(|element| element.dimension() as i32)
        .max()
        .unwrap_or(0);

    writeln!(writer, "$MeshFormat")?;
    writeln!(writer, "4.1 0 8")?;
    writeln!(writer, "$EndMeshFormat")?;
    write_physical_names(writer, mesh)?;

    writeln!(writer, "$Nodes")?;
    if nodes.is_empty() {
        writeln!(writer, "0 0 0 0")?;
    } else {
        writeln!(
            writer,
            "1 {} {} {}",
            nodes.len(),
            min_node_tag,
            max_node_tag
        )?;
        writeln!(writer, "{} 1 0 {}", entity_dim, nodes.len())?;
        for node in &nodes {
            writeln!(writer, "{}", node.id)?;
        }
        for node in &nodes {
            writeln!(
                writer,
                "{} {} {}",
                node.position.x, node.position.y, node.position.z
            )?;
        }
    }
    writeln!(writer, "$EndNodes")?;

    let mut blocks: BTreeMap<(u8, i32, i32), Vec<&rmsh_model::Element>> = BTreeMap::new();
    for element in &elements {
        let gmsh_type = gmsh_type_id(element.etype)?;
        let entity_tag = element.physical_tag.unwrap_or(1);
        blocks
            .entry((element.dimension(), entity_tag, gmsh_type))
            .or_default()
            .push(*element);
    }

    writeln!(writer, "$Elements")?;
    if elements.is_empty() {
        writeln!(writer, "0 0 0 0")?;
    } else {
        writeln!(
            writer,
            "{} {} {} {}",
            blocks.len(),
            elements.len(),
            min_element_tag,
            max_element_tag
        )?;
        for ((dimension, entity_tag, gmsh_type), block_elements) in blocks {
            writeln!(
                writer,
                "{} {} {} {}",
                dimension,
                entity_tag,
                gmsh_type,
                block_elements.len()
            )?;
            for element in block_elements {
                write!(writer, "{}", element.id)?;
                for node_id in &element.node_ids {
                    write!(writer, " {}", node_id)?;
                }
                writeln!(writer)?;
            }
        }
    }
    writeln!(writer, "$EndElements")?;

    Ok(())
}

/// Parse a Gmsh MSH file (v2.2 or v4.1 ASCII) from a reader.
pub fn parse_msh<R: BufRead>(reader: R) -> Result<Mesh, MshError> {
    let mut mesh = Mesh::new();
    let mut lines = reader.lines();
    let mut line_num: usize = 0;
    let mut version = MshVersion::V4;

    let next_line =
        |lines: &mut std::io::Lines<R>, line_num: &mut usize| -> Result<String, MshError> {
            *line_num += 1;
            lines
                .next()
                .ok_or_else(|| MshError::Parse {
                    line: *line_num,
                    message: "Unexpected end of file".into(),
                })?
                .map_err(MshError::Io)
        };

    while let Some(line_result) = lines.next() {
        line_num += 1;
        let line = line_result.map_err(MshError::Io)?;
        let trimmed = line.trim();

        match trimmed {
            "$MeshFormat" => {
                let format_line = next_line(&mut lines, &mut line_num)?;
                let parts: Vec<&str> = format_line.trim().split_whitespace().collect();
                if parts.is_empty() {
                    return Err(MshError::Parse {
                        line: line_num,
                        message: "Empty format line".into(),
                    });
                }
                let ver_str = parts[0];
                if ver_str.starts_with("2.") {
                    version = MshVersion::V2;
                } else if ver_str.starts_with("4.") {
                    version = MshVersion::V4;
                } else {
                    return Err(MshError::UnsupportedVersion(ver_str.into()));
                }

                let end = next_line(&mut lines, &mut line_num)?;
                if end.trim() != "$EndMeshFormat" {
                    return Err(MshError::Parse {
                        line: line_num,
                        message: "Expected $EndMeshFormat".into(),
                    });
                }
            }
            "$PhysicalNames" => {
                let count_line = next_line(&mut lines, &mut line_num)?;
                let count: usize = count_line.trim().parse().map_err(|_| MshError::Parse {
                    line: line_num,
                    message: "Invalid physical names count".into(),
                })?;
                for _ in 0..count {
                    let pn_line = next_line(&mut lines, &mut line_num)?;
                    let parts: Vec<&str> = pn_line.trim().splitn(3, ' ').collect();
                    if parts.len() >= 3 {
                        let dim: i32 = parts[0].parse().unwrap_or(0);
                        let tag: i32 = parts[1].parse().unwrap_or(0);
                        let name = parts[2].trim_matches('"').to_string();
                        mesh.physical_names.insert((dim, tag), name);
                    }
                }
                let end = next_line(&mut lines, &mut line_num)?;
                if end.trim() != "$EndPhysicalNames" {
                    return Err(MshError::Parse {
                        line: line_num,
                        message: "Expected $EndPhysicalNames".into(),
                    });
                }
            }
            "$Nodes" => match version {
                MshVersion::V2 => parse_nodes_v2(&mut lines, &mut line_num, &mut mesh)?,
                MshVersion::V4 => parse_nodes_v4(&mut lines, &mut line_num, &mut mesh)?,
            },
            "$Elements" => match version {
                MshVersion::V2 => parse_elements_v2(&mut lines, &mut line_num, &mut mesh)?,
                MshVersion::V4 => parse_elements_v4(&mut lines, &mut line_num, &mut mesh)?,
            },
            _ => {
                if trimmed.starts_with('$') && !trimmed.starts_with("$End") {
                    let end_tag = format!("$End{}", &trimmed[1..]);
                    loop {
                        let skip_line = next_line(&mut lines, &mut line_num)?;
                        if skip_line.trim() == end_tag {
                            break;
                        }
                    }
                }
            }
        }
    }

    log::info!(
        "Parsed MSH: {} nodes, {} elements",
        mesh.node_count(),
        mesh.element_count()
    );

    Ok(mesh)
}

fn parse_nodes_v2<R: BufRead>(
    lines: &mut std::io::Lines<R>,
    line_num: &mut usize,
    mesh: &mut Mesh,
) -> Result<(), MshError> {
    let header = next_line_raw(lines, line_num)?;
    let num_nodes: usize = header.trim().parse().map_err(|_| MshError::Parse {
        line: *line_num,
        message: "Invalid node count".into(),
    })?;

    for _ in 0..num_nodes {
        let node_line = next_line_raw(lines, line_num)?;
        let parts: Vec<&str> = node_line.trim().split_whitespace().collect();
        if parts.len() < 4 {
            return Err(MshError::Parse {
                line: *line_num,
                message: "Invalid node line, expected: tag x y z".into(),
            });
        }
        let tag: u64 = parts[0].parse().map_err(|_| MshError::Parse {
            line: *line_num,
            message: "Invalid node tag".into(),
        })?;
        let x: f64 = parts[1].parse().map_err(|_| MshError::Parse {
            line: *line_num,
            message: "Invalid node x coordinate".into(),
        })?;
        let y: f64 = parts[2].parse().map_err(|_| MshError::Parse {
            line: *line_num,
            message: "Invalid node y coordinate".into(),
        })?;
        let z: f64 = parts[3].parse().map_err(|_| MshError::Parse {
            line: *line_num,
            message: "Invalid node z coordinate".into(),
        })?;
        mesh.add_node(Node::new(tag, x, y, z));
    }

    let end = next_line_raw(lines, line_num)?;
    if end.trim() != "$EndNodes" {
        return Err(MshError::Parse {
            line: *line_num,
            message: "Expected $EndNodes".into(),
        });
    }

    Ok(())
}

fn parse_elements_v2<R: BufRead>(
    lines: &mut std::io::Lines<R>,
    line_num: &mut usize,
    mesh: &mut Mesh,
) -> Result<(), MshError> {
    let header = next_line_raw(lines, line_num)?;
    let num_elements: usize = header.trim().parse().map_err(|_| MshError::Parse {
        line: *line_num,
        message: "Invalid element count".into(),
    })?;

    for _ in 0..num_elements {
        let elem_line = next_line_raw(lines, line_num)?;
        let parts: Vec<&str> = elem_line.trim().split_whitespace().collect();
        if parts.len() < 3 {
            return Err(MshError::Parse {
                line: *line_num,
                message: "Invalid element line".into(),
            });
        }
        let elem_tag: u64 = parts[0].parse().map_err(|_| MshError::Parse {
            line: *line_num,
            message: "Invalid element tag".into(),
        })?;
        let element_type_id: i32 = parts[1].parse().map_err(|_| MshError::Parse {
            line: *line_num,
            message: "Invalid element type".into(),
        })?;
        let num_tags: usize = parts[2].parse().map_err(|_| MshError::Parse {
            line: *line_num,
            message: "Invalid number of tags".into(),
        })?;

        let node_start = 3 + num_tags;
        if parts.len() < node_start {
            return Err(MshError::Parse {
                line: *line_num,
                message: "Element line too short for tags".into(),
            });
        }
        let node_ids: Vec<u64> = parts[node_start..]
            .iter()
            .map(|s| {
                s.parse::<u64>().map_err(|_| MshError::Parse {
                    line: *line_num,
                    message: "Invalid node id in element".into(),
                })
            })
            .collect::<Result<_, _>>()?;

        let etype = ElementType::from_gmsh_type_id(element_type_id);
        mesh.add_element(Element::new(elem_tag, etype, node_ids));
    }

    let end = next_line_raw(lines, line_num)?;
    if end.trim() != "$EndElements" {
        return Err(MshError::Parse {
            line: *line_num,
            message: "Expected $EndElements".into(),
        });
    }

    Ok(())
}

fn parse_nodes_v4<R: BufRead>(
    lines: &mut std::io::Lines<R>,
    line_num: &mut usize,
    mesh: &mut Mesh,
) -> Result<(), MshError> {
    let header = next_line_raw(lines, line_num)?;
    let parts: Vec<&str> = header.trim().split_whitespace().collect();
    if parts.len() < 4 {
        return Err(MshError::Parse {
            line: *line_num,
            message: "Invalid nodes header".into(),
        });
    }
    let num_entity_blocks: usize = parts[0].parse().unwrap_or(0);

    for _ in 0..num_entity_blocks {
        let block_header = next_line_raw(lines, line_num)?;
        let bp: Vec<&str> = block_header.trim().split_whitespace().collect();
        if bp.len() < 4 {
            return Err(MshError::Parse {
                line: *line_num,
                message: "Invalid node block header".into(),
            });
        }
        let num_in_block: usize = bp[3].parse().unwrap_or(0);

        let mut tags = Vec::with_capacity(num_in_block);
        for _ in 0..num_in_block {
            let tag_line = next_line_raw(lines, line_num)?;
            let tag: u64 = tag_line.trim().parse().map_err(|_| MshError::Parse {
                line: *line_num,
                message: "Invalid node tag".into(),
            })?;
            tags.push(tag);
        }

        for tag in tags {
            let coord_line = next_line_raw(lines, line_num)?;
            let coords: Vec<f64> = coord_line
                .trim()
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if coords.len() >= 3 {
                mesh.add_node(Node::new(tag, coords[0], coords[1], coords[2]));
            }
        }
    }

    let end = next_line_raw(lines, line_num)?;
    if end.trim() != "$EndNodes" {
        return Err(MshError::Parse {
            line: *line_num,
            message: "Expected $EndNodes".into(),
        });
    }

    Ok(())
}

fn parse_elements_v4<R: BufRead>(
    lines: &mut std::io::Lines<R>,
    line_num: &mut usize,
    mesh: &mut Mesh,
) -> Result<(), MshError> {
    let header = next_line_raw(lines, line_num)?;
    let parts: Vec<&str> = header.trim().split_whitespace().collect();
    if parts.len() < 4 {
        return Err(MshError::Parse {
            line: *line_num,
            message: "Invalid elements header".into(),
        });
    }
    let num_entity_blocks: usize = parts[0].parse().unwrap_or(0);

    for _ in 0..num_entity_blocks {
        let block_header = next_line_raw(lines, line_num)?;
        let bp: Vec<&str> = block_header.trim().split_whitespace().collect();
        if bp.len() < 4 {
            return Err(MshError::Parse {
                line: *line_num,
                message: "Invalid element block header".into(),
            });
        }
        let element_type_id: i32 = bp[2].parse().unwrap_or(0);
        let num_in_block: usize = bp[3].parse().unwrap_or(0);
        let etype = ElementType::from_gmsh_type_id(element_type_id);
        let expected_nodes = etype.node_count();

        for _ in 0..num_in_block {
            let elem_line = next_line_raw(lines, line_num)?;
            let values: Vec<u64> = elem_line
                .trim()
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if values.is_empty() {
                continue;
            }
            let elem_tag = values[0];
            let node_ids: Vec<u64> = values[1..].to_vec();

            if expected_nodes > 0 && node_ids.len() != expected_nodes {
                log::warn!(
                    "Element {} (type {:?}): expected {} nodes, got {}",
                    elem_tag,
                    etype,
                    expected_nodes,
                    node_ids.len()
                );
            }

            mesh.add_element(Element::new(elem_tag, etype, node_ids));
        }
    }

    let end = next_line_raw(lines, line_num)?;
    if end.trim() != "$EndElements" {
        return Err(MshError::Parse {
            line: *line_num,
            message: "Expected $EndElements".into(),
        });
    }

    Ok(())
}

fn next_line_raw<R: BufRead>(
    lines: &mut std::io::Lines<R>,
    line_num: &mut usize,
) -> Result<String, MshError> {
    *line_num += 1;
    lines
        .next()
        .ok_or_else(|| MshError::Parse {
            line: *line_num,
            message: "Unexpected end of file".into(),
        })?
        .map_err(MshError::Io)
}

fn validate_mesh(mesh: &Mesh) -> Result<(), MshError> {
    for element in &mesh.elements {
        gmsh_type_id(element.etype)?;
        for node_id in &element.node_ids {
            if !mesh.nodes.contains_key(node_id) {
                return Err(MshError::MissingNode(*node_id));
            }
        }
    }
    Ok(())
}

fn write_physical_names<W: Write>(writer: &mut W, mesh: &Mesh) -> Result<(), MshError> {
    if mesh.physical_names.is_empty() {
        return Ok(());
    }

    let mut physical_names: Vec<_> = mesh.physical_names.iter().collect();
    physical_names.sort_by_key(|((dim, tag), _)| (*dim, *tag));

    writeln!(writer, "$PhysicalNames")?;
    writeln!(writer, "{}", physical_names.len())?;
    for ((dim, tag), name) in physical_names {
        writeln!(writer, "{} {} \"{}\"", dim, tag, name)?;
    }
    writeln!(writer, "$EndPhysicalNames")?;

    Ok(())
}

fn sorted_nodes(mesh: &Mesh) -> Vec<&rmsh_model::Node> {
    let mut nodes: Vec<_> = mesh.nodes.values().collect();
    nodes.sort_by_key(|node| node.id);
    nodes
}

fn sorted_elements(mesh: &Mesh) -> Vec<&rmsh_model::Element> {
    let mut elements: Vec<_> = mesh.elements.iter().collect();
    elements.sort_by_key(|element| element.id);
    elements
}

fn gmsh_type_id(element_type: ElementType) -> Result<i32, MshError> {
    match element_type {
        ElementType::Point1 => Ok(15),
        ElementType::Line2 => Ok(1),
        ElementType::Triangle3 => Ok(2),
        ElementType::Quad4 => Ok(3),
        ElementType::Tetrahedron4 => Ok(4),
        ElementType::Hexahedron8 => Ok(5),
        ElementType::Prism6 => Ok(6),
        ElementType::Pyramid5 => Ok(7),
        ElementType::Unknown(_) => Err(MshError::UnsupportedElementType(element_type)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn sample_mesh() -> Mesh {
        let mut mesh = Mesh::new();
        mesh.add_node(Node::new(1, 0.0, 0.0, 0.0));
        mesh.add_node(Node::new(2, 1.0, 0.0, 0.0));
        mesh.add_node(Node::new(3, 0.0, 1.0, 0.0));
        mesh.add_node(Node::new(4, 0.0, 0.0, 1.0));

        let mut tri = Element::new(1, ElementType::Triangle3, vec![1, 2, 3]);
        tri.physical_tag = Some(11);
        mesh.add_element(tri);
        mesh.add_element(Element::new(2, ElementType::Tetrahedron4, vec![1, 2, 3, 4]));
        mesh.physical_names.insert((2, 11), "surface".to_string());
        mesh
    }

    fn assert_mesh_core_eq(actual: &Mesh, expected: &Mesh) {
        assert_eq!(actual.node_count(), expected.node_count());
        assert_eq!(actual.element_count(), expected.element_count());
        assert_eq!(actual.physical_names, expected.physical_names);

        let mut actual_nodes: Vec<_> = actual.nodes.iter().collect();
        actual_nodes.sort_by_key(|(id, _)| **id);
        let mut expected_nodes: Vec<_> = expected.nodes.iter().collect();
        expected_nodes.sort_by_key(|(id, _)| **id);
        for ((actual_id, actual_node), (expected_id, expected_node)) in
            actual_nodes.into_iter().zip(expected_nodes)
        {
            assert_eq!(actual_id, expected_id);
            assert_eq!(actual_node.position, expected_node.position);
        }

        let mut actual_elements: Vec<_> = actual.elements.iter().collect();
        actual_elements.sort_by_key(|element| element.id);
        let mut expected_elements: Vec<_> = expected.elements.iter().collect();
        expected_elements.sort_by_key(|element| element.id);
        for (actual_element, expected_element) in actual_elements.into_iter().zip(expected_elements)
        {
            assert_eq!(actual_element.id, expected_element.id);
            assert_eq!(actual_element.etype, expected_element.etype);
            assert_eq!(actual_element.node_ids, expected_element.node_ids);
        }
    }

    #[test]
    fn test_parse_simple_msh_v4() {
        let msh_data = r#"$MeshFormat
4.1 0 8
$EndMeshFormat
$Nodes
1 4 1 4
3 1 0 4
1
2
3
4
0.0 0.0 0.0
1.0 0.0 0.0
0.0 1.0 0.0
0.0 0.0 1.0
$EndNodes
$Elements
1 1 1 1
3 1 4 1
1 1 2 3 4
$EndElements
"#;
        let mesh = parse_msh(Cursor::new(msh_data.as_bytes())).unwrap();
        assert_eq!(mesh.node_count(), 4);
        assert_eq!(mesh.element_count(), 1);
        assert_eq!(
            mesh.elements[0].etype,
            rmsh_model::ElementType::Tetrahedron4
        );
    }

    #[test]
    fn test_parse_simple_msh_v2() {
        let msh_data = r#"$MeshFormat
2.2 0 8
$EndMeshFormat
$Nodes
4
1 0.0 0.0 0.0
2 1.0 0.0 0.0
3 0.0 1.0 0.0
4 0.0 0.0 1.0
$EndNodes
$Elements
2
1 2 2 0 1 1 2 3
2 4 2 0 1 1 2 3 4
$EndElements
"#;
        let mesh = parse_msh(Cursor::new(msh_data.as_bytes())).unwrap();
        assert_eq!(mesh.node_count(), 4);
        assert_eq!(mesh.element_count(), 2);
        assert_eq!(mesh.elements[0].etype, rmsh_model::ElementType::Triangle3);
        assert_eq!(
            mesh.elements[1].etype,
            rmsh_model::ElementType::Tetrahedron4
        );
        assert_eq!(mesh.elements[0].node_ids, vec![1, 2, 3]);
        assert_eq!(mesh.elements[1].node_ids, vec![1, 2, 3, 4]);
    }

    #[test]
    fn test_write_roundtrip_msh_v2() {
        let mesh = sample_mesh();
        let mut output = Vec::new();
        write_msh_v2(&mut output, &mesh).unwrap();

        let parsed = parse_msh(Cursor::new(output)).unwrap();
        assert_mesh_core_eq(&parsed, &mesh);
    }

    #[test]
    fn test_write_roundtrip_msh_v4() {
        let mesh = sample_mesh();
        let mut output = Vec::new();
        write_msh_v4(&mut output, &mesh).unwrap();

        let parsed = parse_msh(Cursor::new(output)).unwrap();
        assert_mesh_core_eq(&parsed, &mesh);
    }
}
