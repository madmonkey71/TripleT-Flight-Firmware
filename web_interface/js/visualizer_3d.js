// web_interface/js/visualizer_3d.js

let scene, camera, renderer, cube;

function init3DVisualizer() {
    const container = document.getElementById('visualizer3DContainer');
    if (!container) {
        console.error('3D Visualizer container not found!');
        return;
    }

    // Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x2a2a2a); // Dark grey background

    // Camera
    const width = container.clientWidth || 500; // Default width if not yet rendered
    const height = container.clientHeight || 300; // Default height
    camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
    camera.position.z = 5;
    camera.position.y = 2; // Slightly elevated view

    // Renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    container.appendChild(renderer.domElement);

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 10, 7.5);
    scene.add(directionalLight);

    // 3D Object (e.g., a box representing the rocket body)
    // Dimensions: width, height, depth. Height (Y) is the longest axis for a rocket.
    const geometry = new THREE.BoxGeometry(1, 2, 0.5); 
    const material = new THREE.MeshStandardMaterial({ color: 0x0077ff }); // Blue color
    cube = new THREE.Mesh(geometry, material);
    scene.add(cube);
    
    // Add axes helper for better orientation visualization
    const axesHelper = new THREE.AxesHelper( 3 ); // Length of axes lines
    scene.add( axesHelper );
    
    // Initial render
    animate();

    // Handle window resize
    window.addEventListener('resize', onWindowResize, false);
}

function onWindowResize() {
    const container = document.getElementById('visualizer3DContainer');
    if (!container) return;

    const width = container.clientWidth;
    const height = container.clientHeight || 300; // Ensure a minimum height

    if (camera && renderer) {
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
        renderer.setSize(width, height);
    }
}

function update3DVisualizer(roll, pitch, yaw) {
    if (cube) {
        // Three.js uses radians for rotation.
        // The order of rotation in Euler angles is important.
        // Typically, for aerospace, it's Yaw (Z), Pitch (Y), Roll (X) - intrinsic Tait-Bryan angles.
        // However, Three.js's default .rotation.set(x, y, z) applies them in X, then Y, then Z order.
        // It's often more robust to use Quaternions if direct RPY causes gimbal lock or ordering issues,
        // but for visualization, direct Euler angle setting is often sufficient if angles are within typical ranges.

        // Convert degrees to radians
        const rollRad = THREE.MathUtils.degToRad(roll);
        const pitchRad = THREE.MathUtils.degToRad(pitch);
        const yawRad = THREE.MathUtils.degToRad(yaw);

        // Set rotation. The default order for .rotation.set is X, Y, Z.
        // We need to map our Roll, Pitch, Yaw to the correct axes for the visualizer.
        // Assuming:
        // - Roll is rotation around the object's longest axis (typically X if aligned with world X, or Y if object is upright)
        // - Pitch is up/down nose (typically Y or X)
        // - Yaw is left/right nose (typically Z)
        
        // For a box where Y is "up" (height of the box):
        // Roll around Y-axis (object's local Y)
        // Pitch around X-axis (object's local X)
        // Yaw around Z-axis (object's local Z) - This mapping can be tricky and might need adjustment
        // based on how the axes helper looks relative to the model.

        // Let's try a common mapping:
        // Firmware Roll (often X-axis of sensor) -> Object's X-axis rotation
        // Firmware Pitch (often Y-axis of sensor) -> Object's Y-axis rotation
        // Firmware Yaw (often Z-axis of sensor) -> Object's Z-axis rotation
        // This might need to be permuted depending on sensor frame vs. object frame vs. world frame.
        
        // Standard Three.js .rotation order is X, Y, Z (Euler XYZ).
        // If your firmware RPY corresponds to rotations about X, then Y, then Z of the *sensor frame*:
        cube.rotation.set(pitchRad, yawRad, rollRad, 'ZYX'); // A common order for aerospace: Yaw, Pitch, Roll
        // If the above order 'ZYX' for Euler angles (intrinsic rotations) still looks off,
        // you might need to adjust the order or directly manipulate quaternions.
        // A common source of confusion is extrinsic vs. intrinsic rotations and the order of application.
        // For now, let's use 'ZYX' as it's a standard aerospace sequence.
        // The axes helper will be crucial to verify this.
        // Red = X, Green = Y, Blue = Z for the world axes.
    }
}

function animate() {
    requestAnimationFrame(animate);
    if (renderer && scene && camera) {
        renderer.render(scene, camera);
    }
}

// Expose functions to be called from other scripts
window.init3DVisualizer = init3DVisualizer;
window.update3DVisualizer = update3DVisualizer; 