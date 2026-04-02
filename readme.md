# rmsh

## run viewer as desktop app
```
cargo run -p rmsh-viewer
```

## run viewer as web app

```
trunk serve
```

## run cad examples

```sh
cargo run -p rmsh-cad --example primitives_gallery
cargo run -p rmsh-cad --example boolean_gallery
cargo run -p rmsh-cad --example transform_gallery
cargo run -p rmsh-cad --example step_export_gallery
```

Generated `.msh` and `.step` files are written to `crates/cad/examples/output/` and can be opened with `rmsh-viewer`, Gmsh, or another STEP viewer for visual inspection.