import init, { RobotSimulator } from './pkg/robotics_wasm.js';

class RobotVisualizerApp {
    constructor() {
        this.simulator = null;
        this.canvas = null;
        this.ctx = null;
        this.scale = 80; // pixels per meter
        this.centerX = 400;
        this.centerY = 300;
    }

    async initialize() {
        console.log('Initializing Robot Visualizer...');

        // Initialize WASM
        await init();
        console.log('WASM initialized');

        // Setup canvas
        this.setupCanvas();

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

    createSimulator() {
        const l1 = parseFloat(document.getElementById('link1').value);
        const l2 = parseFloat(document.getElementById('link2').value);

        this.simulator = new RobotSimulator(l1, l2);
        console.log(`Robot created with L1=${l1}, L2=${l2}`);
    }

    setupControls() {
        // Joint angle sliders
        const theta1Slider = document.getElementById('theta1');
        const theta2Slider = document.getElementById('theta2');
        const theta1Value = document.getElementById('theta1-value');
        const theta2Value = document.getElementById('theta2-value');

        theta1Slider.addEventListener('input', (e) => {
            const value = parseInt(e.target.value);
            theta1Value.textContent = value;
            this.updateAngles();
        });

        theta2Slider.addEventListener('input', (e) => {
            const value = parseInt(e.target.value);
            theta2Value.textContent = value;
            this.updateAngles();
        });

        // Update robot button
        document.getElementById('update-robot').addEventListener('click', () => {
            this.createSimulator();
            this.updateAngles();
        });

        // Preset buttons
        const presets = {
            'zero': { theta1: 0, theta2: 0 },
            'right': { theta1: 45, theta2: 90 },
            'folded': { theta1: 90, theta2: -90 },
            'stretched': { theta1: 0, theta2: 180 }
        };

        document.querySelectorAll('.preset-btn[data-preset]').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const presetName = e.target.getAttribute('data-preset');
                const preset = presets[presetName];

                if (preset) {
                    document.getElementById('theta1').value = preset.theta1;
                    document.getElementById('theta2').value = preset.theta2;
                    document.getElementById('theta1-value').textContent = preset.theta1;
                    document.getElementById('theta2-value').textContent = preset.theta2;
                    this.updateAngles();
                }
            });
        });
    }

    updateAngles() {
        const theta1 = parseFloat(document.getElementById('theta1').value);
        const theta2 = parseFloat(document.getElementById('theta2').value);

        // Convert degrees to radians for the simulator
        this.simulator.set_angles(
            theta1 * Math.PI / 180,
            theta2 * Math.PI / 180
        );

        this.render();
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
        const positions = this.simulator.get_joint_positions();

        if (!positions || positions.length === 0) {
            console.error('No positions returned from simulator');
            return;
        }

        // Convert world coordinates (3D) to canvas coordinates (2D)
        // Ignoring Z coordinate for now (planar robots have Z=0)
        const canvasPositions = positions.map(pos => ({
            x: this.centerX + pos.x * this.scale,
            y: this.centerY - pos.y * this.scale // Flip Y for canvas
            // pos.z ignored (always 0 for planar robots)
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
