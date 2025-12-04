# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Rust + WebAssembly (WASM) robotics simulator prototype ("Robo-Desmos") designed to run high-performance robotics calculations in the browser. The project compiles Rust code to WASM and exposes functions to JavaScript for a client-side robotics playground.

**Phase 2 (Current):** Migrated from custom 2D transforms to nalgebra Matrix4 for 3D homogeneous transformations.

## Build Commands

### Building the WASM Module
```bash
wasm-pack build --target web
```
This compiles Rust code to WebAssembly and generates JavaScript glue code in `pkg/`. The `pkg/` directory is regenerated on every build and should not be manually edited.

### Running the Application
```bash
python3 -m http.server
```
Then open http://localhost:8000 in a browser. WASM files must be served via HTTP (not file://) due to browser security restrictions.

### Standard Rust Commands
```bash
cargo build          # Build the project
cargo check          # Check for errors without building
cargo clippy         # Run linter
cargo test           # Run tests (if any exist)
```

## Architecture

### Core Structure
- **src/lib.rs**: Main WASM entry point. Functions marked with `#[wasm_bindgen]` are exposed to JavaScript.
- **src/geometry3d.rs**: 3D geometry primitives using nalgebra::Matrix4 for homogeneous transformations.
- **src/robot.rs**: Robot arm configuration (link lengths, joint angles).
- **src/kinematics.rs**: Forward kinematics computation using 3D transforms.
- **Cargo.toml**: Configured with `crate-type = ["cdylib"]` to produce a WASM binary. Uses `wasm-bindgen` and `nalgebra`.
- **index.html**: Frontend that imports WASM module as ES6 module from `./pkg/robotics_wasm.js`.

### Rust-JavaScript Bridge
- Functions exposed via `#[wasm_bindgen]` macro become callable from JavaScript
- Browser console API is accessed via extern "C" declaration with `#[wasm_bindgen(js_namespace = console)]`
- The init function from wasm-bindgen must be called before using any Rust functions
- Complex data (positions) serialized with serde-wasm-bindgen

### Current Implementation (Phase 2)
The codebase implements a 2-DOF planar robot arm with:
1. 3D homogeneous transforms using nalgebra Matrix4
2. Forward kinematics returning 3D positions (Z=0 for planar)
3. Interactive sliders for joint angle control
4. Real-time Canvas 2D visualization

### Future Roadmap
The project plans to evolve through these phases:
1. ✅ Phase 1: 2-DOF planar robot with custom 2D transforms
2. ✅ Phase 2: Migrate to nalgebra for 3D math
3. Phase 2b: Implement DH parameters
4. Phase 2c: Support arbitrary-DOF robots
5. Phase 3: Integrate Three.js for 3D visualization
6. Phase 4: Implement inverse kinematics solver

Current dependencies include `nalgebra` for matrix operations. Future may include `wgpu` for GPU acceleration.

## Development Notes

### When Adding New Rust Functions
- Mark functions with `#[wasm_bindgen]` to expose them to JavaScript
- Rebuild with `wasm-pack build --target web` after changes
- Supported parameter types: primitives, String slices (&str), and some Rust types with proper annotations
- Return types should be String, primitives, or properly annotated structs

### Working with Console Logging
Use the existing `log()` function defined in lib.rs for browser console output:
```rust
log(&format!("Debug: {}", value));
```

### File Organization
- Keep all Rust source code in `src/`
- `pkg/` is auto-generated - never commit or manually edit
- `target/` contains Rust build artifacts - ignored by git
