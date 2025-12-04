# Robo-Desmos (WASM Prototype)

This is the foundational prototype for a "Desmos-like" robotics playground built with **Rust** and **WebAssembly (WASM)**.

The goal of this project is to create a high-performance, client-side robotics simulator that allows users to visualize kinematics, manipulate robot arms, and experiment with robotics math directly in the browser—without complex installations or backend servers.

## 🚀 Tech Stack

- **Core Logic:** Rust (compiled to WASM)
- **Frontend:** HTML/JavaScript (Native ES Modules)
- **Build Tool:** `wasm-pack`
- **Math Backend:** `nalgebra` (Planned) / `wgpu` (Planned)

## 🛠 Prerequisites

Before running this project, ensure you have the following installed on your machine:

1.  **Rust & Cargo:** [Install Rust](https://www.rust-lang.org/tools/install)
2.  **wasm-pack:** The tool to compile Rust to WebAssembly.
    ```bash
    cargo install wasm-pack
    ```
3.  **Python 3:** (Or any simple local web server) to serve the files.

## ⚙️ Setup & Installation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/eshafeeqe/robotics-wasm.git
    cd robotics-wasm
    ```

2.  **Build the WASM module:**
    This compiles the Rust code into a `.wasm` binary and generates the JavaScript glue code in the `pkg/` folder.
    ```bash
    wasm-pack build --target web
    ```

## ▶️ How to Run

Browsers strictly block WASM files from loading directly from the hard drive for security reasons. You must use a local server.

1.  **Start a local server:**
    ```bash
    python3 -m http.server
    ```

2.  **Open in Browser:**
    Go to [http://localhost:8000](http://localhost:8000).

3.  **Test:**
    Enter text in the input box and click "Run Rust Code". Check the browser console (F12) to see logs coming directly from the Rust backend.

## 📂 Project Structure

```text
├── Cargo.toml      # Rust configuration and dependencies
├── src/
│   └── lib.rs      # Main Rust source code (WASM entry point)
├── index.html      # Frontend UI
├── pkg/            # (Generated) Compiled WASM and JS glue code
└── .gitignore      # Git ignore rules

Future Roadmap

Phase 1: Port Basic Spatial Math (Homogeneous Transforms) to Rust.

Phase 2: Implement DH-Parameter solver.

Phase 3: Integrate Three.js / React-Three-Fiber for 3D visualization.

Phase 4: Implement Inverse Kinematics solver.