// This file is required by the index.html file and will
// be executed in the renderer process for that window.
// No Node.js APIs are available in this process because
// `nodeIntegration` is turned off. Use `preload.js` to
// selectively enable features needed in the rendering
// process.
const THREE = require("three")

// const statsRenderer = new Stats();
// statsRenderer.dom.style.cssText = '';
// document.getElementById('statsRenderer').appendChild(statsRenderer.dom);
// const statsKinect = new Stats();
// statsKinect.dom.style.cssText = '';
// document.getElementById('statsKinect').appendChild(statsKinect.dom);

const KinectAzure = require('kinect-azure');
const kinect = new KinectAzure();

const canvas = document.getElementById('outputCanvas');
canvas.width = canvas.clientWidth;
canvas.height = canvas.clientHeight;

const renderer = new THREE.WebGLRenderer({
    canvas
});
renderer.setPixelRatio(window.devicePixelRatio);

const scene = new THREE.Scene();

const camera = new THREE.PerspectiveCamera(30, canvas.clientWidth / canvas.clientHeight, 1, 10000);
camera.position.set(0, 0, 2000);
camera.lookAt(0, 0, 0);
// const controls = new THREE.OrbitControls(camera, renderer.domElement);

const geom = new THREE.Geometry();
const material = new THREE.PointsMaterial({
    vertexColors: THREE.VertexColors
});

const DEPTH_WIDTH = 640;
const DEPTH_HEIGHT = 576;
const numPoints = DEPTH_WIDTH * DEPTH_HEIGHT;
for (let i = 0; i < numPoints; i++) {
    const x = (i % DEPTH_WIDTH) - DEPTH_WIDTH * 0.5;
    const y = DEPTH_HEIGHT / 2 - Math.floor(i / DEPTH_WIDTH);
    const particle = new THREE.Vector3(x, y, 0);
    geom.vertices.push(particle);
    const color = new THREE.Color(0x000000);
    geom.colors.push(color);
}

const cloud = new THREE.Points(geom, material);
scene.add(cloud);

const depthModeRange = kinect.getDepthModeRange(KinectAzure.K4A_DEPTH_MODE_NFOV_UNBINNED);

if (kinect.open()) {
    kinect.startCameras({
        depth_mode: KinectAzure.K4A_DEPTH_MODE_NFOV_UNBINNED,
        color_format: KinectAzure.K4A_IMAGE_FORMAT_COLOR_BGRA32,
        color_resolution: KinectAzure.K4A_COLOR_RESOLUTION_720P,
        include_color_to_depth: true
    });
    kinect.startListening((data) => {
        // statsKinect.update();
        const newDepthData = Buffer.from(data.depthImageFrame.imageData);
        const newColorData = Buffer.from(data.colorToDepthImageFrame.imageData);
        let pointIndex = 0;


        for (let i = 0; i < newDepthData.length; i += 2) {
            const depthValue = newDepthData[i + 1] << 8 | newDepthData[i];
            const b = newColorData[pointIndex * 4 + 0];
            const g = newColorData[pointIndex * 4 + 1];
            const r = newColorData[pointIndex * 4 + 2];
            if (depthValue > depthModeRange.min && depthValue < depthModeRange.max) {
                geom.vertices[pointIndex].z = depthValue;
            } else {
                geom.vertices[pointIndex].z = Number.MAX_VALUE;
            }
            geom.colors[pointIndex].setRGB(r / 255, g / 255, b / 255);
            pointIndex++;
        }
        geom.verticesNeedUpdate = true;
        geom.colorsNeedUpdate = true;
    });
}

const resize = () => {
    renderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
    camera.aspect = canvas.clientWidth / canvas.clientHeight;
    camera.updateProjectionMatrix();
};
window.addEventListener('resize', resize);

const animate = () => {
    // statsRenderer.update();
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
}

// expose the kinect instance to the window object in this demo app to allow the parent window to close it between sessions
window.kinect = kinect;
animate();