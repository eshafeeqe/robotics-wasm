import init, { RobotSimulator } from './pkg/robotics_wasm.js';

class RobotVisualizerApp {
    constructor() {
        this.simulator = null;
        this.canvas = null;
        this.ctx = null;
        this.scale = 80; // pixels per meter
        this.centerX = 400;
        this.centerY = 300;

        // Configuration state
        this.mode = 'simple'; // 'simple' or 'dh'
        this.numJoints = 2;
        this.linkLengths = [2.0, 1.5];
        this.dhParams = [];
        this.jointAngles = [0, 0];
    }

    async initialize() {
        console.log('Initializing Robot Visualizer...');

        // Initialize WASM
        await init();
        console.log('WASM initialized');

        // Setup canvas
        this.setupCanvas();

        // Generate initial UI
        this.generateUI();

        // Create initial robot
        this.createSimulator();

        // Setup event listeners
        this.setupControls();

        // Initial render
        this.render();

        console.log('Robot Visualizer ready!');
    }

    setupCanvas() {
        this.canvas = document.getElementById('robot-canvas');
        this.ctx = this.canvas.getContext('2d');

        // Set canvas dimensions
        this.centerX = this.canvas.width / 2;
        this.centerY = this.canvas.height / 2;
    }

    generateUI() {
        this.generateLinkInputs();
        this.generateDHInputs();
        this.generateJointAngleSliders();
        this.updatePresetButtons();
    }

    generateLinkInputs() {
        const container = document.getElementById('link-inputs-container');
        container.innerHTML = '';

        for (let i = 0; i < this.numJoints; i++) {
            const linkDiv = document.createElement('div');
            linkDiv.className = 'link-input-item';

            const defaultLength = this.linkLengths[i] || 1.5;

            linkDiv.innerHTML = `
                <label for="link-${i}">Link ${i + 1} (m)</label>
                <input type="number" id="link-${i}" min="0.1" max="5" step="0.1" value="${defaultLength}">
            `;

            container.appendChild(linkDiv);
        }
    }

    generateDHInputs() {
        const container = document.getElementById('dh-inputs-container');
        container.innerHTML = '';

        for (let i = 0; i < this.numJoints; i++) {
            const dhJoint = document.createElement('div');
            dhJoint.className = 'dh-joint';

            // Default DH parameters for planar robot
            const defaultA = this.linkLengths[i] || 1.5;
            const defaultAlpha = 0.0;
            const defaultD = 0.0;
            const defaultTheta = 0.0;

            dhJoint.innerHTML = `
                <div class="dh-joint-header">Joint ${i + 1}</div>
                <div class="dh-param-grid">
                    <div class="dh-param-item">
                        <label>a (Link Length)</label>
                        <input type="number" id="dh-a-${i}" step="0.1" value="${defaultA}">
                    </div>
                    <div class="dh-param-item">
                        <label>α (Twist, rad)</label>
                        <input type="number" id="dh-alpha-${i}" step="0.1" value="${defaultAlpha}">
                    </div>
                    <div class="dh-param-item">
                        <label>d (Offset)</label>
                        <input type="number" id="dh-d-${i}" step="0.1" value="${defaultD}">
                    </div>
                    <div class="dh-param-item">
                        <label>θ (Angle, rad)</label>
                        <input type="number" id="dh-theta-${i}" step="0.1" value="${defaultTheta}">
                    </div>
                </div>
                <div class="dh-joint-type">
                    <label>
                        <input type="radio" name="joint-type-${i}" value="Revolute" checked>
                        Revolute
                    </label>
                    <label>
                        <input type="radio" name="joint-type-${i}" value="Prismatic">
                        Prismatic
                    </label>
                </div>
            `;

            container.appendChild(dhJoint);
        }
    }

    generateJointAngleSliders() {
        const container = document.getElementById('joint-angles-container');
        container.innerHTML = '';

        for (let i = 0; i < this.numJoints; i++) {
            const angleDiv = document.createElement('div');
            angleDiv.className = 'control-group';

            const currentAngle = this.jointAngles[i] || 0;

            angleDiv.innerHTML = `
                <label>
                    <span>θ${i + 1} (deg)</span>
                    <span class="value-display" id="theta${i}-value">${currentAngle}</span>
                </label>
                <input type="range" id="theta${i}" min="-180" max="180" value="${currentAngle}" step="1">
            `;

            container.appendChild(angleDiv);

            // Add event listener
            const slider = angleDiv.querySelector('input[type="range"]');
            const valueDisplay = angleDiv.querySelector('.value-display');

            slider.addEventListener('input', (e) => {
                const value = parseInt(e.target.value);
                valueDisplay.textContent = value;
                this.jointAngles[i] = value;
                this.updateAngles();
            });
        }
    }

    updatePresetButtons() {
        const presetContainer = document.getElementById('preset-buttons');
        // Only show presets for 2-DOF robots
        presetContainer.style.display = this.numJoints === 2 ? 'grid' : 'none';
    }

    setupControls() {
        // Mode selector
        document.querySelectorAll('input[name="robot-mode"]').forEach(radio => {
            radio.addEventListener('change', (e) => {
                this.mode = e.target.value;
                this.updateModeUI();
            });
        });

        // Number of joints slider
        const numJointsSlider = document.getElementById('num-joints');
        const numJointsValue = document.getElementById('num-joints-value');

        numJointsSlider.addEventListener('input', (e) => {
            const value = parseInt(e.target.value);
            numJointsValue.textContent = value;
            this.numJoints = value;

            // Resize arrays
            while (this.linkLengths.length < this.numJoints) {
                this.linkLengths.push(1.5);
            }
            while (this.jointAngles.length < this.numJoints) {
                this.jointAngles.push(0);
            }

            // Regenerate UI
            this.generateUI();
        });

        // Update robot button
        document.getElementById('update-robot').addEventListener('click', () => {
            this.createSimulator();
            this.updateAngles();
        });

        // Preset buttons (only for 2-DOF)
        const presets = {
            'zero': { angles: [0, 0] },
            'right': { angles: [45, 90] },
            'folded': { angles: [90, -90] },
            'stretched': { angles: [0, 180] }
        };

        document.querySelectorAll('.preset-btn[data-preset]').forEach(btn => {
            btn.addEventListener('click', (e) => {
                if (this.numJoints !== 2) return;

                const presetName = e.target.getAttribute('data-preset');
                const preset = presets[presetName];

                if (preset) {
                    for (let i = 0; i < 2; i++) {
                        document.getElementById(`theta${i}`).value = preset.angles[i];
                        document.getElementById(`theta${i}-value`).textContent = preset.angles[i];
                        this.jointAngles[i] = preset.angles[i];
                    }
                    this.updateAngles();
                }
            });
        });
    }

    updateModeUI() {
        const simpleConfig = document.getElementById('simple-config');
        const dhConfig = document.getElementById('dh-config');

        if (this.mode === 'simple') {
            simpleConfig.style.display = 'block';
            dhConfig.style.display = 'none';
        } else {
            simpleConfig.style.display = 'none';
            dhConfig.style.display = 'block';
        }
    }

    createSimulator() {
        try {
            if (this.mode === 'simple') {
                // Read link lengths from UI
                const lengths = [];
                for (let i = 0; i < this.numJoints; i++) {
                    const input = document.getElementById(`link-${i}`);
                    lengths.push(parseFloat(input.value));
                }
                this.linkLengths = lengths;

                // Create simple robot
                if (this.numJoints === 2) {
                    // Use legacy constructor for backwards compatibility
                    this.simulator = new RobotSimulator(lengths[0], lengths[1]);
                } else {
                    // Use new_simple for arbitrary-DOF
                    this.simulator = RobotSimulator.new_simple(lengths);
                }

                console.log(`Created ${this.numJoints}-DOF simple robot with lengths:`, lengths);
            } else {
                // Read DH parameters from UI
                const dhParams = [];
                for (let i = 0; i < this.numJoints; i++) {
                    const a = parseFloat(document.getElementById(`dh-a-${i}`).value);
                    const alpha = parseFloat(document.getElementById(`dh-alpha-${i}`).value);
                    const d = parseFloat(document.getElementById(`dh-d-${i}`).value);
                    const theta = parseFloat(document.getElementById(`dh-theta-${i}`).value);
                    const jointType = document.querySelector(`input[name="joint-type-${i}"]:checked`).value;

                    dhParams.push({
                        a,
                        alpha,
                        d,
                        theta,
                        joint_type: jointType,
                        joint_offset: jointType === 'Revolute' ? theta : d
                    });
                }

                // Create DH robot
                this.simulator = RobotSimulator.new_dh(dhParams);
                console.log(`Created ${this.numJoints}-DOF DH robot`);
            }
        } catch (error) {
            console.error('Failed to create robot:', error);
            alert('Failed to create robot: ' + error);
        }
    }

    updateAngles() {
        if (!this.simulator) return;

        try {
            // Convert degrees to radians
            const anglesRad = this.jointAngles.map(deg => deg * Math.PI / 180);

            if (this.numJoints === 2) {
                // Use legacy method for 2-DOF
                this.simulator.set_angles(anglesRad[0], anglesRad[1]);
            } else {
                // Use new array method for arbitrary-DOF
                this.simulator.set_angles_array(anglesRad);
            }

            this.render();
        } catch (error) {
            console.error('Failed to update angles:', error);
        }
    }

    render() {
        this.clearCanvas();
        this.drawGrid();
        this.drawAxes();
        this.drawRobot();
        this.updateInfo();
    }

    clearCanvas() {
        this.ctx.fillStyle = '#ffffff';
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
    }

    drawGrid() {
        const ctx = this.ctx;
        const gridSize = 50;

        ctx.strokeStyle = '#f0f0f0';
        ctx.lineWidth = 1;

        // Vertical lines
        for (let x = 0; x <= this.canvas.width; x += gridSize) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, this.canvas.height);
            ctx.stroke();
        }

        // Horizontal lines
        for (let y = 0; y <= this.canvas.height; y += gridSize) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(this.canvas.width, y);
            ctx.stroke();
        }
    }

    drawAxes() {
        const ctx = this.ctx;
        const axisLength = 100;

        // X-axis (red)
        ctx.strokeStyle = '#e74c3c';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(this.centerX, this.centerY);
        ctx.lineTo(this.centerX + axisLength, this.centerY);
        ctx.stroke();

        // X-axis arrow
        ctx.fillStyle = '#e74c3c';
        ctx.beginPath();
        ctx.moveTo(this.centerX + axisLength, this.centerY);
        ctx.lineTo(this.centerX + axisLength - 8, this.centerY - 5);
        ctx.lineTo(this.centerX + axisLength - 8, this.centerY + 5);
        ctx.fill();

        // Y-axis (green)
        ctx.strokeStyle = '#27ae60';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(this.centerX, this.centerY);
        ctx.lineTo(this.centerX, this.centerY - axisLength);
        ctx.stroke();

        // Y-axis arrow
        ctx.fillStyle = '#27ae60';
        ctx.beginPath();
        ctx.moveTo(this.centerX, this.centerY - axisLength);
        ctx.lineTo(this.centerX - 5, this.centerY - axisLength + 8);
        ctx.lineTo(this.centerX + 5, this.centerY - axisLength + 8);
        ctx.fill();

        // Labels
        ctx.fillStyle = '#555';
        ctx.font = '14px sans-serif';
        ctx.fillText('X', this.centerX + axisLength + 10, this.centerY + 5);
        ctx.fillText('Y', this.centerX + 5, this.centerY - axisLength - 10);
    }

    drawRobot() {
        if (!this.simulator) return;

        const positions = this.simulator.get_joint_positions();

        if (!positions || positions.length === 0) {
            console.error('No positions returned from simulator');
            return;
        }

        // Convert world coordinates (3D) to canvas coordinates (2D)
        const canvasPositions = positions.map(pos => ({
            x: this.centerX + pos.x * this.scale,
            y: this.centerY - pos.y * this.scale // Flip Y for canvas
        }));

        // Draw links
        this.ctx.lineWidth = 8;
        for (let i = 0; i < canvasPositions.length - 1; i++) {
            const from = canvasPositions[i];
            const to = canvasPositions[i + 1];

            // Gradient from blue to red
            const gradient = this.ctx.createLinearGradient(from.x, from.y, to.x, to.y);
            gradient.addColorStop(0, '#3498db');
            gradient.addColorStop(1, '#e74c3c');

            this.ctx.strokeStyle = gradient;
            this.ctx.beginPath();
            this.ctx.moveTo(from.x, from.y);
            this.ctx.lineTo(to.x, to.y);
            this.ctx.stroke();
        }

        // Draw joints
        canvasPositions.forEach((pos, index) => {
            if (index === 0) {
                // Base
                this.drawJoint(pos.x, pos.y, 10, '#2c3e50');
            } else if (index === canvasPositions.length - 1) {
                // End-effector
                this.drawJoint(pos.x, pos.y, 12, '#e74c3c');
            } else {
                // Intermediate joints
                this.drawJoint(pos.x, pos.y, 8, '#3498db');
            }
        });
    }

    drawJoint(x, y, radius, color) {
        this.ctx.fillStyle = color;
        this.ctx.beginPath();
        this.ctx.arc(x, y, radius, 0, 2 * Math.PI);
        this.ctx.fill();

        // White outline
        this.ctx.strokeStyle = '#ffffff';
        this.ctx.lineWidth = 2;
        this.ctx.stroke();
    }

    updateInfo() {
        if (!this.simulator) return;

        const endEffector = this.simulator.get_end_effector_position();

        if (endEffector) {
            const x = endEffector.x;
            const y = endEffector.y;
            const z = endEffector.z;
            const distance = Math.sqrt(x * x + y * y + z * z);

            document.getElementById('pos-x').textContent = x.toFixed(3);
            document.getElementById('pos-y').textContent = y.toFixed(3);
            document.getElementById('pos-z').textContent = z.toFixed(3);
            document.getElementById('pos-dist').textContent = distance.toFixed(3);
        }
    }
}

// Initialize the app when the page loads
const app = new RobotVisualizerApp();
app.initialize().catch(err => {
    console.error('Failed to initialize app:', err);
});
