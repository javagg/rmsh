use anyhow::{Context, Result};
use regex::Regex;
use std::collections::BTreeMap;
use std::fs;
use std::path::PathBuf;

#[derive(Debug, Clone)]
struct ApiFn {
    c_name: String,
    rust_name: String,
    py_export_name: String,
    py_path: String,
    params: Vec<String>,
}

fn main() {
    if let Err(err) = run() {
        panic!("rmsh-py build script failed: {err:#}");
    }
}

fn run() -> Result<()> {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR")?);
    let header_path = manifest_dir.join("api").join("rmshc.h");
    let header = fs::read_to_string(&header_path)
        .with_context(|| format!("failed to read {}", header_path.display()))?;

    println!("cargo:rerun-if-changed={}", header_path.display());

    let functions = parse_header(&header)?;
    if functions.is_empty() {
        anyhow::bail!("no API function found in {}", header_path.display());
    }

    let out_dir = PathBuf::from(std::env::var("OUT_DIR")?);
    let rust_out = out_dir.join("generated_bindings.rs");
    fs::write(&rust_out, generate_rust(&functions))
        .with_context(|| format!("failed to write {}", rust_out.display()))?;

    let py_out = manifest_dir.join("python").join("rmsh.py");
    fs::write(&py_out, generate_python(&functions))
        .with_context(|| format!("failed to write {}", py_out.display()))?;

    Ok(())
}

fn parse_header(header: &str) -> Result<Vec<ApiFn>> {
    let re = Regex::new(r"RMSH_API\s+int\s+(rmsh[A-Za-z0-9_]+)\s*\(([^;]*)\)\s*;")?;
    let mut out = Vec::new();

    for cap in re.captures_iter(header) {
        let c_name = cap[1].to_string();
        let args = cap[2].trim();
        let params = parse_param_names(args);
        let py_path = c_name_to_py_path(&c_name);
        let py_export_name = snake_case(&py_path.replace('.', "_"));
        let rust_name = format!("{}_impl", py_export_name);

        out.push(ApiFn {
            c_name,
            rust_name,
            py_export_name,
            py_path,
            params,
        });
    }

    Ok(out)
}

fn parse_param_names(args: &str) -> Vec<String> {
    if args.is_empty() || args == "void" {
        return Vec::new();
    }

    args.split(',')
        .map(str::trim)
        .filter(|a| !a.is_empty())
        .map(param_name)
        .collect()
}

fn param_name(arg: &str) -> String {
    let part = arg.split('=').next().unwrap_or(arg).trim();
    let mut token = String::new();
    for ch in part.chars().rev() {
        if ch.is_ascii_alphanumeric() || ch == '_' {
            token.push(ch);
        } else if !token.is_empty() {
            break;
        }
    }
    let mut token: String = token.chars().rev().collect();
    if token.is_empty() {
        token = "arg".to_string();
    }
    token
}

fn c_name_to_py_path(c_name: &str) -> String {
    let body = c_name.strip_prefix("rmsh").unwrap_or(c_name);

    if let Some(rest) = body.strip_prefix("ModelMesh") {
        return format!("model.mesh.{}", lower_camel(rest));
    }
    if let Some(rest) = body.strip_prefix("ModelOcc") {
        return format!("model.occ.{}", lower_camel(rest));
    }
    if let Some(rest) = body.strip_prefix("ModelGeo") {
        return format!("model.geo.{}", lower_camel(rest));
    }
    if let Some(rest) = body.strip_prefix("Model") {
        return format!("model.{}", lower_camel(rest));
    }
    if let Some(rest) = body.strip_prefix("Option") {
        return format!("option.{}", lower_camel(rest));
    }
    if let Some(rest) = body.strip_prefix("Logger") {
        return format!("logger.{}", lower_camel(rest));
    }
    if let Some(rest) = body.strip_prefix("Plugin") {
        return format!("plugin.{}", lower_camel(rest));
    }
    if let Some(rest) = body.strip_prefix("Gui") {
        return format!("gui.{}", lower_camel(rest));
    }

    lower_camel(body)
}

fn lower_camel(s: &str) -> String {
    let mut chars = s.chars();
    match chars.next() {
        Some(c) => format!("{}{}", c.to_ascii_lowercase(), chars.collect::<String>()),
        None => String::new(),
    }
}

fn snake_case(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 8);
    let mut prev_is_lower_or_digit = false;
    for ch in s.chars() {
        if ch.is_ascii_uppercase() {
            if prev_is_lower_or_digit {
                out.push('_');
            }
            out.push(ch.to_ascii_lowercase());
            prev_is_lower_or_digit = false;
        } else {
            out.push(ch);
            prev_is_lower_or_digit = ch.is_ascii_lowercase() || ch.is_ascii_digit();
        }
    }
    out
}

fn generate_rust(functions: &[ApiFn]) -> String {
    let mut out = String::new();
    out.push_str("use pyo3::exceptions::PyNotImplementedError;\n");
    out.push_str("use pyo3::types::{PyDict, PyTuple};\n\n");

    for f in functions {
        out.push_str("#[pyfunction]\n");
        out.push_str(&format!("#[pyo3(name = \"{}\", signature = (*args, **kwargs))]\n", f.py_export_name));
        out.push_str(&format!(
            "fn {}(args: &pyo3::Bound<'_, PyTuple>, kwargs: Option<&pyo3::Bound<'_, PyDict>>) -> pyo3::PyResult<()> {{\n",
            f.rust_name
        ));
        out.push_str("    let _ = (args, kwargs);\n");
        out.push_str(&format!(
            "    Err(PyNotImplementedError::new_err(\"{} is not implemented yet\"))\n",
            f.c_name
        ));
        out.push_str("}\n\n");
    }

    out.push_str("pub fn register_generated(m: &pyo3::Bound<'_, pyo3::types::PyModule>) -> pyo3::PyResult<()> {\n");
    for f in functions {
        out.push_str(&format!(
            "    m.add_function(pyo3::wrap_pyfunction!({}, m)?)?;\n",
            f.rust_name
        ));
    }
    out.push_str("    Ok(())\n}");

    out
}

fn generate_python(functions: &[ApiFn]) -> String {
    let mut out = String::new();
    out.push_str("\"\"\"Auto-generated rmsh Python API.\n\n");
    out.push_str("This file is generated from api/rmshc.h by crates/rmsh-py/build.rs.\n");
    out.push_str("Do not edit manually.\n\"\"\"\n\n");
    out.push_str("from __future__ import annotations\n\n");
    out.push_str("import _rmsh\n\n");
    out.push_str("__version__ = getattr(_rmsh, \"__version__\", \"0.0.0\")\n\n");
    out.push_str("def _invoke(symbol: str, *args, **kwargs):\n");
    out.push_str("    return getattr(_rmsh, symbol)(*args, **kwargs)\n\n");
    out.push_str("class _Namespace:\n");
    out.push_str("    def __init__(self, name: str):\n");
    out.push_str("        self._name = name\n\n");

    let mut ns_tree: BTreeMap<String, Vec<&ApiFn>> = BTreeMap::new();
    let mut top: Vec<&ApiFn> = Vec::new();

    for f in functions {
        let parts: Vec<&str> = f.py_path.split('.').collect();
        if parts.len() == 1 {
            top.push(f);
            continue;
        }
        let ns = parts[..parts.len() - 1].join(".");
        ns_tree.entry(ns).or_default().push(f);
    }

    top.sort_by(|a, b| a.py_path.cmp(&b.py_path));
    for f in top {
        let py_name = f.py_path.as_str();
        out.push_str(&format!("def {}(*args, **kwargs):\n", py_name));
        out.push_str(&format!("    return _invoke(\"{}\", *args, **kwargs)\n\n", f.py_export_name));
    }

    let mut namespace_keys: Vec<String> = ns_tree.keys().cloned().collect();
    namespace_keys.sort();

    for key in &namespace_keys {
        let class_name = ns_class_name(key);
        out.push_str(&format!("class {}(_Namespace):\n", class_name));
        out.push_str("    pass\n\n");
    }

    for key in &namespace_keys {
        let class_name = ns_class_name(key);
        let var_name = key.replace('.', "_");
        out.push_str(&format!("{} = {}(\"{}\")\n", var_name, class_name, key));
    }
    out.push('\n');

    let mut attach_lines = Vec::new();
    for key in &namespace_keys {
        if let Some((parent, child)) = key.rsplit_once('.') {
            let parent_var = parent.replace('.', "_");
            let child_var = key.replace('.', "_");
            attach_lines.push(format!("{}.{} = {}", parent_var, child, child_var));
        }
    }
    attach_lines.sort();
    for line in &attach_lines {
        out.push_str(line);
        out.push('\n');
    }
    if !attach_lines.is_empty() {
        out.push('\n');
    }

    for (ns, funcs) in &ns_tree {
        let ns_var = ns.replace('.', "_");
        let mut sorted_funcs = funcs.clone();
        sorted_funcs.sort_by(|a, b| a.py_path.cmp(&b.py_path));
        for f in sorted_funcs {
            let py_name = f.py_path.split('.').next_back().unwrap_or("api");
            let params = if f.params.is_empty() {
                "".to_string()
            } else {
                f.params.join(", ")
            };
            out.push_str(&format!(
                "def _{}_{}(*args, **kwargs):\n",
                ns_var, py_name
            ));
            out.push_str(&format!(
                "    \"\"\"{}({})\"\"\"\n",
                f.c_name,
                params
            ));
            out.push_str(&format!(
                "    return _invoke(\"{}\", *args, **kwargs)\n\n",
                f.py_export_name
            ));
            out.push_str(&format!("{}.{} = _{}_{}\n\n", ns_var, py_name, ns_var, py_name));
        }
    }

    out.push_str("__all__ = [\n");
    for f in functions {
        if !f.py_path.contains('.') {
            out.push_str(&format!("    \"{}\",\n", f.py_path));
        }
    }
    out.push_str("    \"model\",\n");
    out.push_str("    \"option\",\n");
    out.push_str("    \"logger\",\n");
    out.push_str("    \"plugin\",\n");
    out.push_str("    \"gui\",\n");
    out.push_str("]\n");

    out.push_str("\n# Public namespace aliases\n");
    out.push_str("model = model\n");
    out.push_str("option = option\n");
    out.push_str("logger = logger\n");
    out.push_str("plugin = plugin\n");
    out.push_str("gui = gui\n");

    out
}

fn ns_class_name(ns: &str) -> String {
    let mut out = String::from("_Ns");
    for part in ns.split('.') {
        let mut chars = part.chars();
        if let Some(first) = chars.next() {
            out.push(first.to_ascii_uppercase());
            out.push_str(chars.as_str());
        }
    }
    out
}
