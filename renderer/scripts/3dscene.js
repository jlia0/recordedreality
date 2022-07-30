// const THREE = require("three");

const cv = require('../assets/opencv');
const KinectAzure = require("kinect-azure");
const kinect = new KinectAzure();

let flag = false;
let newDepthData, newColorData;
let cvImageData = null;


const
    statsRenderer = new Stats();
statsRenderer.dom.style.cssText = '';
document.getElementById('statsRenderer').appendChild(statsRenderer.dom);
const statsKinect = new Stats();
statsKinect.dom.style.cssText = '';
document.getElementById('statsKinect').appendChild(statsKinect.dom);
let labelRenderer;

function openCvReady() {
    cv['onRuntimeInitialized'] = () => {
        // do all your work here

        flag = true;
        processImg();
    };
}

const canvas = document.getElementById('outputCanvas');
canvas.width = canvas.clientWidth;
canvas.height = canvas.clientHeight;

const $colorToDepthCanvas = document.getElementById('colorToDepthCanvas'),
    colorToDepthCtx = $colorToDepthCanvas.getContext('2d');
let colorToDepthImageData;

const renderer = new THREE.WebGLRenderer({
    canvas
});
renderer.setPixelRatio(window.devicePixelRatio);

const scene = new THREE.Scene();

const camera = new THREE.OrthographicCamera(canvas.clientWidth / -5, canvas.clientWidth / 5, canvas.clientHeight / 5, canvas.clientHeight / -5, 1, 10000);
// const camera = new THREE.PerspectiveCamera(75, canvas.clientWidth / canvas.clientHeight, 1, 10000)
camera.position.set(0, 0, -1000);
// camera.lookAt(new THREE.Vector3(0, 0, 0));
camera.lookAt(scene.position)
const controls = new THREE.MapControls(camera, renderer.domElement);

const geom = new THREE.Geometry();
const material = new THREE.PointsMaterial({
    size: 0.05,
    vertexColors: true,
    // transparent: true
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

geom.center();
const cloud = new THREE.Points(geom, material);

scene.add(cloud);

function initLabel(){

    // Label

    const earthGeometry = new THREE.SphereGeometry(1, 16, 16);
    const earthMaterial = new THREE.MeshPhongMaterial({
        specular: 0x333333,
        shininess: 5,
        normalScale: new THREE.Vector2(0.85, 0.85)
    });
    const earth = new THREE.Mesh(earthGeometry, earthMaterial);

    earth.layers.enableAll();

    const earthDiv = document.createElement('div');
    earthDiv.className = 'label';
    earthDiv.textContent = 'Earth';
    earthDiv.style.marginTop = '-1em';
    const earthLabel = new THREE.CSS2DObject(earthDiv);
    earthLabel.position.set(0, 0, 500);
    earth.add(earthLabel);
    earthLabel.layers.set(0);

    labelRenderer = new THREE.CSS2DRenderer();
    labelRenderer.setSize(window.innerWidth, window.innerHeight);
    labelRenderer.domElement.style.position = 'absolute';
    labelRenderer.domElement.style.top = '0px';
    document.body.appendChild(labelRenderer.domElement);

    scene.add(earth);

}


const depthModeRange = kinect.getDepthModeRange(KinectAzure.K4A_DEPTH_MODE_NFOV_UNBINNED);

if (kinect.open()) {
    kinect.startCameras({
        depth_mode: KinectAzure.K4A_DEPTH_MODE_NFOV_UNBINNED,
        color_format: KinectAzure.K4A_IMAGE_FORMAT_COLOR_BGRA32,
        color_resolution: KinectAzure.K4A_COLOR_RESOLUTION_720P,
        include_color_to_depth: true
    });
    kinect.startListening((data) => {
        statsKinect.update();
        newDepthData = Buffer.from(data.depthImageFrame.imageData);
        newColorData = Buffer.from(data.colorToDepthImageFrame.imageData);
        let pointIndex = 0;
        for (let i = 0; i < newDepthData.length; i += 2) {
            const depthValue = newDepthData[i + 1] << 8 | newDepthData[i];
            const b = newColorData[pointIndex * 4 + 0];
            const g = newColorData[pointIndex * 4 + 1];
            const r = newColorData[pointIndex * 4 + 2];
            if (depthValue > depthModeRange.min && depthValue < depthModeRange.max) {
                geom.vertices[pointIndex].z = depthValue / 2.5;
            } else {
                geom.vertices[pointIndex].z = Number.MAX_VALUE;
            }
            geom.colors[pointIndex].setRGB(r / 255, g / 255, b / 255);
            pointIndex++;
        }
        geom.verticesNeedUpdate = true;
        geom.colorsNeedUpdate = true;

        // render color to depth
        if (!colorToDepthImageData && data.colorToDepthImageFrame.width > 0) {
            $colorToDepthCanvas.width = data.colorToDepthImageFrame.width;
            $colorToDepthCanvas.height = data.colorToDepthImageFrame.height;
            colorToDepthImageData = colorToDepthCtx.createImageData($colorToDepthCanvas.width, $colorToDepthCanvas.height);
        }
        if (colorToDepthImageData) {
            renderBGRA32ColorFrame(colorToDepthCtx, colorToDepthImageData, data.colorToDepthImageFrame);
        }
    });
}




function processImg() {
    let colors = [
        { id: 0, low: [180, 20, 40, 255], hight: [210, 40, 60, 255] }, // red
        { id: 1, low: [230, 200, 80, 255], hight: [255, 230, 110, 255] }, // yellow
    ]

    let begin = Date.now();

    for (let i = 0; i < colors.length; i++) {
        let color = colors[i]
        detectColor(color)
    }

    let delay = 1000 / 30 - (Date.now() - begin);
    setTimeout(processImg, delay);

    function detectColor(color) {
        try {
            if (flag && cvImageData !== null) {
                // (231,79,86,255)

                // let colorCanvas = document.getElementById("colorToDepthCanvas");
                // let colorCtx = colorCanvas.getContext('2d');
                // let colorImgData = colorCtx.getImageData(0, 0, colorCanvas.width, colorCanvas.height);

                let src = cv.matFromImageData(cvImageData);
                let dst = cv.matFromImageData(cvImageData);

                // const lowRed = [180, 20, 40, 255];
                // const highRed = [210, 40, 60, 255];

                let low = new cv.Mat(src.rows, src.cols, src.type(), color.low);
                let high = new cv.Mat(src.rows, src.cols, src.type(), color.hight);

                // if lower bound and upper bound are set
                if (low !== null && high !== null) {
                    // filter out contours
                    cv.inRange(src, low, high, dst);

                    let contours = new cv.MatVector();
                    let hierarchy = new cv.Mat();
                    // get remaining contours and hierachy
                    cv.findContours(dst, contours, hierarchy, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE);

                    // find the largest are of contours
                    let maxArea = 0;
                    let maxCnt = null;

                    for (let i = 0; i < contours.size(); i++) {
                        let cnt = contours.get(i);
                        let area = cv.contourArea(cnt, false);

                        if (area > maxArea) {
                            maxArea = area
                            maxCnt = cnt
                        }
                    }

                    // if there is a contour exist in the frame, draw
                    if (maxCnt && maxCnt.data32S) {

                        let toDraw = new cv.MatVector();
                        toDraw.push_back(maxCnt);
                        let cvColor = new cv.Scalar(255, 0, 0);

                        getCenterPoint(maxCnt.data32S, color.id);

                        // draw the contours
                        for (let i = 0; i < toDraw.size(); ++i) {
                            cv.drawContours(dst, toDraw, i, cvColor, 15, cv.LINE_8, new cv.Mat(), 0);
                        }
                    }

                }

                cv.imshow("opencvCanvas", dst);
                src.delete();
                dst.delete();
                low.delete();
                high.delete();


            }
        } catch (e) {
            console.log(e)
        }

    }
}

let sphereGeometry = new THREE.SphereGeometry(20, 20, 20)
let sphereMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff })
let sphere = new THREE.Mesh(sphereGeometry, sphereMaterial)
scene.add(sphere)

let sphere2 = new THREE.Mesh(sphereGeometry, sphereMaterial)
scene.add(sphere2)

let linePoints = [new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0)]
let lineGeometry = new THREE.CylinderGeometry()
let line = new THREE.Line(lineGeometry, sphereMaterial)
scene.add(line)

let textMesh = new THREE.Mesh()
let fontLoader = new THREE.FontLoader()
fontLoader.load('https://unpkg.com/three@0.120.0/examples/fonts/helvetiker_regular.typeface.json', function(font) {
    let textGeometry = new THREE.TextGeometry('Hello World', {
        font: font,
        size: 10,
        height: 2,
        curveSegments: 12,
        bevelThickness: 1,
        bevelSize: 1,
        bevelEnabled: true
    })

    let textMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff })
    textMesh = new THREE.Mesh(textGeometry, textMaterial)
    scene.add(textMesh)
})

let trajectory = new THREE.Points();
scene.add(trajectory)

let currentMode = 1
textMesh.visible = false
line.visible = false
sphere.visible = false
sphere2.visible = false


function getCenterPoint(points, id) {
    //  calculating the center
    let sumX = 0;
    let sumY = 0;
    let numPoints = points.length / 2;
    for (let i = 0; i < points.length; i += 2) {
        sumX += points[i];
        sumY += points[i + 1];
    }

    let centerX = Math.floor(sumX / numPoints);
    let centerY = Math.floor(sumY / numPoints);
    let pos = DEPTH_WIDTH * centerY + centerX
    let depthZ = geom.vertices[pos].z;

    //console.log(geom.vertices, centerX, centerY, depthZ)

    const actualX = (pos % DEPTH_WIDTH) - DEPTH_WIDTH * 0.5;
    const actualY = DEPTH_HEIGHT / 2 - Math.floor(pos / DEPTH_WIDTH);

    if (currentMode === 3) {
        trajectory.visible = false
        line.visible = false
        sphere.visible = false
        sphere2.visible = true
        textMesh.visible = true
        if (id === 1) {
            sphere2.position.set(actualX, actualY, depthZ)
            linePoints[1] = new THREE.Vector3(actualX, actualY, depthZ)
            textMesh.position.set(actualX, actualY + 20, depthZ)
        }
        return
    }

    if (currentMode === 2) {
        trajectory.visible = false
        line.visible = true
        sphere.visible = true
        sphere2.visible = true
        textMesh.visible = false
        if (id === 0) {
            sphere.position.set(actualX, actualY, depthZ)
            linePoints[0] = new THREE.Vector3(actualX, actualY, depthZ)
        }
        if (id === 1) {
            sphere2.position.set(actualX, actualY, depthZ)
            linePoints[1] = new THREE.Vector3(actualX, actualY, depthZ)
        }

        createLine(linePoints)
        return

        function createLine(points) {
            let direction = new THREE.Vector3().subVectors(points[1], points[0])
            line.material = new THREE.MeshBasicMaterial({color: 0xffffff})
            line.geometry = new THREE.CylinderGeometry(1, 1, direction.length(), 10, 30, true)
            line.geometry.applyMatrix(new THREE.Matrix4().makeTranslation(0, direction.length() / 2, 0))
            line.geometry.applyMatrix(new THREE.Matrix4().makeRotationX(Math.PI / 2))
            // let mesh = new THREE.Mesh(geometry, material)
            line.position.copy(points[0])
            line.lookAt(points[1])
            return line
        }

    }

    let traj = new THREE.Geometry();
    let traj_material = new THREE.PointsMaterial({
        size: 5,
        vertexColors: true,
        // transparent: true
    });

    const part = new THREE.Vector3(actualX, actualY, depthZ);
    traj.vertices.push(part);
    const color = new THREE.Color(0xFF0000);
    traj.colors.push(color);
    // traj.verticesNeedUpdate = true;

    trajectory = new THREE.Points(traj, traj_material);
    scene.add(trajectory)

    if (depthZ > 10) {
        document.getElementById("pos").innerHTML = `(${centerX}, ${centerY}, ${depthZ})`;
    }
}

const resize = () => {
    renderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
    // labelRenderer.setSize(canvas.clientWidth, canvas.clientHeight );

    camera.aspect = canvas.clientWidth / canvas.clientHeight;
    camera.updateProjectionMatrix();
};
window.addEventListener('resize', resize);

const animate = () => {
    statsRenderer.update();
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
    // labelRenderer.render( scene, camera );
}

let cvFlag = false;

const renderBGRA32ColorFrame = (ctx, canvasImageData, imageFrame) => {
    const newPixelData = Buffer.from(imageFrame.imageData);
    const pixelArray = canvasImageData.data;
    for (let i = 0; i < canvasImageData.data.length; i += 4) {
        pixelArray[i] = newPixelData[i + 2];
        pixelArray[i + 1] = newPixelData[i + 1];
        pixelArray[i + 2] = newPixelData[i];
        pixelArray[i + 3] = 0xff;
    }
    ctx.putImageData(canvasImageData, 0, 0);
    cvImageData = canvasImageData;
    if (flag && cvFlag === false) {
        cvFlag = true;
        console.log("fire up color tracking")
        processImg();
    }

};

// expose the kinect instance to the window object in this demo app to allow the parent window to close it between sessions
window.kinect = kinect;
openCvReady();
animate();
