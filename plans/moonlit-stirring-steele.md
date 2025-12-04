# Phase 2 Plan: nalgebra Migration (Clean Break)

## Overview

Migrate the robotics-wasm project from custom 2D Transform2D to nalgebra's Matrix4 for 3D homogeneous transformations. This provides a solid foundation for future DH parameters and arbitrary-DOF support.

## Scope

**Phase 2 (This Phase):**
- ✅ nalgebra Migration (Matrix4 for 3D transforms)
- ✅ Clean break from Phase 1 API (no backwards compatibility)
- ✅ Keep 2-DOF robot, but use 3D math

**Future Phases:**
- ⏸️ Phase 2b: DH Parameter Implementation
- ⏸️ Phase 2c: Arbitrary-DOF Support
- ⏸️ Phase 3: 3D Visualization (Three.js)

## Critical Design Decisions

### 1. Standard DH Convention
**Decision:** Use Standard (Classic) DH Parameters

The transformation is: `T(i-1,i) = Rot(Z, θ) * Trans(Z, d) * Trans(X, a) * Rot(X, α)`

**Rationale:** More widely taught, better documented, easier to find reference implementations.

### 2. nalgebra Strategy
**Decision:** Use `nalgebra::Matrix4<f64>` directly

**Why Matrix4:**
- Explicit 4x4 homogeneous transformation matrices
- Direct control, matches DH math exactly
- Easy to understand and debug

**Why NOT Isometry3:**
- Uses dual quaternions (less transparent)
- Harder to validate against textbook DH formulas

### 3. Clean Break Strategy
**Decision:** Remove Transform2D completely, no backwards compatibility

**Migration:**
- Delete `src/geometry.rs` entirely
- Replace with `src/geometry3d.rs` using nalgebra
- Update all references in one go
- Simpler, cleaner codebase

### 4. 2D Robot with 3D Math
**Decision:** Keep 2-DOF planar robot, but compute in 3D

**Strategy:**
- Internal: Use 3D transforms (Matrix4), planar robots have Z=0
- WASM: Return 3D positions (x, y, z)
- Frontend: Simple orthographic projection (ignore Z for now)

**Why:**
- Prepares codebase for future 3D robots
- No DH parameters yet (just link lengths + angles)
- Same UI as Phase 1 (2 sliders)

### 5. WASM Interface
**Keep Similar API:**

```rust
#[wasm_bindgen]
impl RobotSimulator {
    pub fn new(link1_length: f64, link2_length: f64) -> Self
    pub fn set_angles(&mut self, theta1: f64, theta2: f64)
    pub fn get_joint_positions(&self) -> JsValue  // Now returns 3D
    pub fn get_end_effector_position(&self) -> JsValue  // Now with Z
}
```

**Changes from Phase 1:**
- Positions now have Z coordinate (always 0 for planar)
- Internal math uses Matrix4 instead of 3x3

## Architecture

### New Module Structure

```
src/
├── lib.rs              # WASM interface (minor updates)
├── geometry3d.rs       # NEW: Replaces geometry.rs, uses nalgebra::Matrix4
├── robot.rs            # UPDATED: Use 3D transforms
├── kinematics.rs       # UPDATED: 3D FK, still 2-DOF
└── (geometry.rs DELETED)
```

### Key Implementations

#### geometry3d.rs (NEW)
```rust
use nalgebra::Matrix4;

pub struct Point3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

pub struct Transform3D {
    matrix: Matrix4<f64>,
}

impl Transform3D {
    pub fn identity() -> Self
    pub fn rotation_x/y/z(angle: f64) -> Self
    pub fn translation(x, y, z: f64) -> Self
    pub fn compose(&self, other: &Transform3D) -> Self
    pub fn transform_point(&self, point: &Point3D) -> Point3D
}
```

#### robot.rs (UPDATED)
```rust
pub struct RobotArm {
    pub link_lengths: Vec<f64>,
    pub joint_angles: Vec<f64>,
}

impl RobotArm {
    pub fn new(link_lengths: Vec<f64>) -> Self
    pub fn set_joint_angles(&mut self, angles: Vec<f64>)
}
```
*Note: Still simple link lengths, DH parameters come later*

#### kinematics.rs (UPDATED)
```rust
pub struct JointPosition3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

pub fn forward_kinematics(robot: &RobotArm) -> Vec<JointPosition3D> {
    // Chain DH transforms in 3D
    // Return all joint positions including end-effector
}
```

## Implementation Plan

### Step 1: Update Dependencies (30 min)
1. Update Cargo.toml - Add `nalgebra = "0.33"`
2. Remove unused serde-wasm-bindgen if not needed

### Step 2: Create geometry3d.rs (2-3 hours)
3. Create src/geometry3d.rs with nalgebra
4. Implement Point3D, Transform3D wrapper
5. Write unit tests (identity, rotations, translations, composition)

### Step 3: Delete geometry.rs (30 min)
6. Delete src/geometry.rs
7. Remove all imports of old Transform2D

### Step 4: Update robot.rs (1 hour)
8. Update to use Transform3D instead of Transform2D
9. Keep same simple API (link_lengths + joint_angles)

### Step 5: Update kinematics.rs (2-3 hours)
10. Change to use 3D transforms
11. Return JointPosition3D (with z: 0.0 for planar)
12. Update tests to expect 3D positions

### Step 6: Update lib.rs (1 hour)
13. Update WASM interface to return 3D positions
14. Keep same API signature

### Step 7: Update Frontend (1-2 hours)
15. Update app.js to handle Z coordinate (ignore it for now)
16. Ensure rendering still works

### Step 8: Testing & Validation (2-3 hours)
17. Run all tests: `cargo test`
18. Build WASM: `wasm-pack build --target web`
19. Test in browser - verify same behavior as Phase 1
20. Check WASM size

### Step 9: Documentation (1 hour)
21. Update README.md - Note nalgebra migration
22. Update CLAUDE.md - New module structure
23. Code cleanup: `cargo fmt`, `cargo clippy`

## Testing Strategy

### Unit Tests (Rust)

**Coverage:**
- Geometry3D: 8 tests (identity, rotations, translations, composition)
- DH Parameters: 6 tests (individual params, full transforms)
- Kinematics: 8 tests (2-DOF, 3-DOF, spatial robots)
- Regression: 3 tests (Phase 1 compatibility)

**Target: 14+ passing tests** (same as Phase 1, but updated for 3D)

### Validation Cases

**2-DOF Planar (must match Phase 1):**
- [0°, 0°] → (3.5, 0.0, **0.0**)  ← Now with Z coordinate
- [90°, 0°] → (0.0, 3.5, **0.0**)
- [90°, 90°] → (-1.5, 2.0, **0.0**)

## Frontend Changes

### Minimal UI Changes

**Update app.js:**
```javascript
// Before: pos.x, pos.y
const canvasPositions = positions.map(pos => ({
    x: this.centerX + pos.x * this.scale,
    y: this.centerY - pos.y * this.scale
    // Ignore pos.z for now (always 0 for planar robots)
}));
```

**Update info panel:**
```javascript
// Show Z coordinate even though it's 0
document.getElementById('pos-x').textContent = x.toFixed(3);
document.getElementById('pos-y').textContent = y.toFixed(3);
document.getElementById('pos-z').textContent = z.toFixed(3);  // NEW
```

**Add Z display to HTML:**
```html
<div class="info-row">
    <span class="info-label">Z:</span>
    <span class="info-value" id="pos-z">0.000</span>
</div>
```

## Dependencies Update

```toml
[dependencies]
wasm-bindgen = "0.2"
nalgebra = "0.33"
serde = { version = "1.0", features = ["derive"] }
serde-wasm-bindgen = "0.6"
```

## Timeline

**Total: 1-2 days (10-15 hours)**

- Day 1: nalgebra integration, geometry3d.rs, update modules
- Day 2: Testing, validation, documentation

## Success Criteria

Phase 2 complete when:

1. **Technical:**
   - nalgebra integrated, WASM < 500KB
   - geometry.rs deleted, geometry3d.rs working
   - 14+ unit tests passing (ported from Phase 1)
   - All math matches Phase 1 results

2. **Functional:**
   - 2-DOF robot still works in browser
   - Positions now include Z coordinate (0.0)
   - UI displays X, Y, Z

3. **Quality:**
   - Same visual behavior as Phase 1
   - Clean codebase (no deprecated code)
   - Documentation updated

## Critical Files

**New Files:**
- `src/geometry3d.rs` - 3D transforms using nalgebra::Matrix4

**Deleted Files:**
- `src/geometry.rs` - Replaced by geometry3d.rs

**Modified Files:**
- `src/kinematics.rs` - Use 3D transforms, return JointPosition3D
- `src/lib.rs` - Return 3D positions from WASM
- `src/robot.rs` - Use Transform3D internally
- `app.js` - Handle Z coordinate in rendering
- `index.html` - Add Z coordinate to info panel
- `Cargo.toml` - Add nalgebra dependency

## Next Steps (Future Phases)

**Phase 2b - DH Parameters:**
- Add dh_parameters.rs module
- Implement Standard DH convention
- Support both planar and spatial robots

**Phase 2c - Arbitrary-DOF:**
- Vector-based WASM API
- Dynamic UI for N joints
- Preset robot configurations

**Phase 3 - 3D Visualization:**
- Migrate to Three.js
- 3D camera controls
- Proper 3D rendering
