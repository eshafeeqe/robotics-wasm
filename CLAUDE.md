# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Rust + WebAssembly (WASM) robotics simulator prototype ("Robo-Desmos") designed to run high-performance robotics calculations in the browser. The project compiles Rust code to WASM and exposes functions to JavaScript for a client-side robotics playground.

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
- **Cargo.toml**: Configured with `crate-type = ["cdylib"]` to produce a WASM binary. Uses `wasm-bindgen` as the Rust-JS bridge.
- **index.html**: Frontend that imports WASM module as ES6 module from `./pkg/robotics_wasm.js`.

### Rust-JavaScript Bridge
- Functions exposed via `#[wasm_bindgen]` macro become callable from JavaScript
- Browser console API is accessed via extern "C" declaration with `#[wasm_bindgen(js_namespace = console)]`
- The init function from wasm-bindgen must be called before using any Rust functions

### Current Implementation
The codebase currently contains a simple "hello world" function (`hello_robotics`) that demonstrates:
1. Receiving input from JavaScript
2. Logging to browser console from Rust
3. Returning strings back to JavaScript

### Future Roadmap
The project plans to evolve through these phases:
1. Port spatial math (homogeneous transforms) to Rust
2. Implement DH-parameter solver
3. Integrate Three.js / React-Three-Fiber for 3D visualization
4. Implement inverse kinematics solver

Future dependencies will likely include `nalgebra` for matrix operations and potentially `wgpu` for GPU acceleration.

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
