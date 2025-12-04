# Robo-Desmos (WASM Prototype)

This is an interactive **2-DOF planar robot arm simulator** built with **Rust** and **WebAssembly (WASM)**. It demonstrates real-time forward kinematics with a Desmos-like interface, where you can manipulate joint angles and instantly see the robot's configuration update.

The goal of this project is to create a high-performance, client-side robotics simulator that allows users to visualize kinematics, manipulate robot arms, and experiment with robotics math directly in the browser—without complex installations or backend servers.

## 🎯 Current Features (MVP)

- **Interactive 2-DOF planar robot arm** with configurable link lengths
- **Real-time forward kinematics** calculated in Rust/WASM
- **Smooth canvas visualization** with color-coded joints and links
- **Interactive sliders** for joint angle control (-180° to 180°)
- **Preset configurations** (Zero, Right Angle, Folded, Stretched)
- **Live end-effector position** display (X, Y, distance from origin)
- **Clean, Desmos-inspired UI** focused on the robotics

## 🚀 Tech Stack

- **Core Logic:** Rust (compiled to WASM)
- **Frontend:** HTML/JavaScript (Native ES6 Modules)
- **Build Tool:** `wasm-pack`
- **Math:** nalgebra (3D homogeneous transformation matrices)
- **Visualization:** Canvas 2D API

## 🛠 Prerequisites

Before running this project, ensure you have the following installed on your machine:

1. **Rust & Cargo:** [Install Rust](https://www.rust-lang.org/tools/install)
2. **wasm-pack:** The tool to compile Rust to WebAssembly.
   ```bash
   cargo install wasm-pack
   ```
3. **Python 3:** (Or any simple local web server) to serve the files.

## ⚙️ Setup & Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/eshafeeqe/robotics-wasm.git
   cd robotics-wasm
   ```

2. **Build the WASM module:**
   This compiles the Rust code into a `.wasm` binary and generates the JavaScript glue code in the `pkg/` folder.
   ```bash
   wasm-pack build --target web
   ```

## ▶️ How to Run

Browsers strictly block WASM files from loading directly from the hard drive for security reasons. You must use a local server.

1. **Start a local server:**
   ```bash
   python3 -m http.server
   ```

2. **Open in Browser:**
   Go to [http://localhost:8000](http://localhost:8000).

3. **Interact with the robot:**
   - Use the sliders to adjust joint angles θ1 and θ2
   - Click preset buttons for common configurations
   - Change link lengths and click "Update Robot"
   - Watch the robot move in real-time!

## 🧪 Testing

Run the Rust unit tests to verify the math:

```bash
cargo test
```

All tests should pass (covering geometry transformations, DH parameters, robot configuration, and forward kinematics).

## 📂 Project Structure

```text
├── Cargo.toml              # Rust configuration and dependencies
├── src/
│   ├── lib.rs              # WASM interface and main entry point
│   ├── geometry3d.rs       # 3D geometry with nalgebra::Matrix4
│   ├── robot.rs            # Robot arm configuration (simple + DH modes)
│   ├── dh_parameters.rs    # Denavit-Hartenberg parameter system
│   └── kinematics.rs       # Forward kinematics algorithm (3D + DH)
├── index.html              # Main UI with canvas and controls
├── app.js                  # JavaScript application logic
├── styles.css              # Clean, Desmos-inspired styling
├── pkg/                    # (Generated) Compiled WASM and JS glue code
└── CLAUDE.md               # Development guide for Claude Code
```

## 🎓 How It Works

### The Math

The simulator uses **nalgebra Matrix4** for 3D homogeneous transformation matrices to compute forward kinematics:

1. Start at the origin (base position)
2. For each joint:
   - Rotate by the joint angle θᵢ around Z-axis (planar motion)
   - Translate along the link length Lᵢ in X direction
   - Record the new joint position in 3D (x, y, z)
   - Compose transformations for the next joint

The result is a chain of positions from base → joint1 → joint2 → end-effector.

**Phase 2 Update:** All calculations now use 3D transforms with nalgebra, even though planar robots have Z=0.

**Phase 2b Update:** Added full Denavit-Hartenberg parameter support using Standard DH convention. Robots can now be configured using either:
- Simple mode: Just link lengths (legacy planar robots)
- DH mode: Full 4-parameter specification (a, α, d, θ) for arbitrary robot configurations

### The Pipeline

```
User Input (Sliders)
    ↓
JavaScript (app.js)
    ↓
WASM Interface (lib.rs)
    ↓
Rust FK Calculation (kinematics.rs)
    ↓
Transform Math (geometry.rs)
    ↓
Return Positions to JS
    ↓
Canvas Rendering
    ↓
Visual Feedback!
```

## 🔮 Future Roadmap

### Phase 1: ✅ MVP Complete
- 2-DOF planar robot arm
- Forward kinematics with homogeneous transforms
- Interactive canvas visualization

### Phase 2: ✅ nalgebra Migration Complete
- Migrated to `nalgebra` Matrix4 for 3D math
- Clean break from custom 2D transforms
- 3D positions (Z always 0 for planar robots)
- Foundation for future 3D robots

### Phase 2b: ✅ DH Parameters Complete
- Implemented Denavit-Hartenberg (DH) parameter system
- Support for both simple planar and DH-based robots
- Standard DH convention: Rot(Z,θ) * Trans(Z,d) * Trans(X,a) * Rot(X,α)
- Dual-mode forward kinematics (legacy planar + DH-based)

### Phase 2c: Extended Features (Next)
- Support arbitrary-DOF robots (3+ joints)
- Add workspace visualization
- Expose DH parameter configuration in UI

### Phase 3: 3D Visualization
- Integrate Three.js / React-Three-Fiber
- 3D spatial manipulator (6-DOF)
- Camera controls and multiple viewports

### Phase 4: Inverse Kinematics & Path Planning
- Inverse kinematics solver
- Path planning visualization
- Trajectory animation
- Collision detection

## 🤝 Contributing

This is an educational prototype. Feel free to:
- Experiment with different robot configurations
- Add new features (3rd DOF, trace paths, etc.)
- Improve the UI/UX
- Optimize the math or rendering

## 📝 License

This project is for educational purposes. Feel free to use and modify as needed.

## 🙏 Acknowledgments

Inspired by [Desmos](https://www.desmos.com/) and the need for accessible robotics education tools.
