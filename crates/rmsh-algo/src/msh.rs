use std::io::BufRead;

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
}

/// Parse a Gmsh MSH v4.1 ASCII file from a reader.
pub fn parse_msh<R: BufRead>(reader: R) -> Result<Mesh, MshError> {
    let mut mesh = Mesh::new();
    let mut lines = reader.lines();
    let mut line_num: usize = 0;

    let next_line = |lines: &mut std::io::Lines<R>, line_num: &mut usize| -> Result<String, MshError> {
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
                let version = parts[0];
                if !version.starts_with("4.") {
                    return Err(MshError::UnsupportedVersion(version.into()));
                }
                // file-type: 0 = ASCII
                // data-size: typically 8 (double)
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
            "$Nodes" => {
                parse_nodes_v4(&mut lines, &mut line_num, &mut mesh)?;
            }
            "$Elements" => {
                parse_elements_v4(&mut lines, &mut line_num, &mut mesh)?;
            }
            _ => {
                // Skip unknown sections — read until matching $End
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

/// Parse $Nodes section in MSH 4.1 format.
/// Format:
///   numEntityBlocks numNodes minNodeTag maxNodeTag
///   entityDim entityTag parametric numNodesInBlock
///     nodeTag
///     ...
///     x y z
///     ...
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
    let _num_nodes: usize = parts[1].parse().unwrap_or(0);

    for _ in 0..num_entity_blocks {
        let block_header = next_line_raw(lines, line_num)?;
        let bp: Vec<&str> = block_header.trim().split_whitespace().collect();
        if bp.len() < 4 {
            return Err(MshError::Parse {
                line: *line_num,
                message: "Invalid node block header".into(),
            });
        }
        let _entity_dim: i32 = bp[0].parse().unwrap_or(0);
        let _entity_tag: i32 = bp[1].parse().unwrap_or(0);
        let _parametric: i32 = bp[2].parse().unwrap_or(0);
        let num_in_block: usize = bp[3].parse().unwrap_or(0);

        // Read node tags
        let mut tags = Vec::with_capacity(num_in_block);
        for _ in 0..num_in_block {
            let tag_line = next_line_raw(lines, line_num)?;
            let tag: u64 = tag_line.trim().parse().map_err(|_| MshError::Parse {
                line: *line_num,
                message: "Invalid node tag".into(),
            })?;
            tags.push(tag);
        }

        // Read coordinates
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

/// Parse $Elements section in MSH 4.1 format.
/// Format:
///   numEntityBlocks numElements minElementTag maxElementTag
///   entityDim entityTag elementType numElementsInBlock
///     elementTag nodeTag ...
///     ...
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
        let _entity_dim: i32 = bp[0].parse().unwrap_or(0);
        let _entity_tag: i32 = bp[1].parse().unwrap_or(0);
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn test_parse_simple_msh() {
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
        let cursor = Cursor::new(msh_data);
        let mesh = parse_msh(cursor.lines().collect::<Result<Vec<_>, _>>().unwrap().join("\n").as_bytes()).unwrap();
        assert_eq!(mesh.node_count(), 4);
        assert_eq!(mesh.element_count(), 1);
        assert_eq!(mesh.elements[0].etype, rmsh_model::ElementType::Tetrahedron4);
    }
}
