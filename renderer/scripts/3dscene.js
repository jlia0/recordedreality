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

const camera = new THREE.PerspectiveCamera(30, canvas.clientWidth / canvas.clientHeight, 1, 10000);
camera.position.set(0, 0, 5000);
camera.lookAt(new THREE.Vector3(0, 0, 0));
const controls = new THREE.OrbitControls(camera, renderer.domElement);

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
                geom.vertices[pointIndex].z = depthValue;
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

    try {
        if (flag && cvImageData !== null) {

            let begin = Date.now();
            // (231,79,86,255)

            // let colorCanvas = document.getElementById("colorToDepthCanvas");
            // let colorCtx = colorCanvas.getContext('2d');
            // let colorImgData = colorCtx.getImageData(0, 0, colorCanvas.width, colorCanvas.height);

            let src = cv.matFromImageData(cvImageData);
            let dst = cv.matFromImageData(cvImageData);


            const lowRed = [180, 10, 5, 255];
            const highRed = [255, 100, 100, 255];

            let low = new cv.Mat(src.rows, src.cols, src.type(), lowRed);
            let high = new cv.Mat(src.rows, src.cols, src.type(), highRed);

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
                    let color = new cv.Scalar(255, 0, 0);

                    getCenterPoint(maxCnt.data32S);

                    // draw the contours
                    for (let i = 0; i < toDraw.size(); ++i) {
                        cv.drawContours(dst, toDraw, i, color, 15, cv.LINE_8, new cv.Mat(), 0);
                    }
                }

            }

            cv.imshow("opencvCanvas", dst);
            src.delete();
            dst.delete();
            low.delete();
            high.delete();


            let delay = 1000 / 30 - (Date.now() - begin);
            setTimeout(processImg, delay);
        }
    } catch (e) {
        console.log(e)
    }


}

function getCenterPoint(points) {
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
    let pos = (DEPTH_WIDTH * DEPTH_HEIGHT) - (centerX * centerY);
    let depthZ = geom.vertices[pos].z;

    //console.log(geom.vertices, centerX, centerY, depthZ)

    if (depthZ > 10){
        document.getElementById("pos").innerHTML = `(${centerX}, ${centerY}, ${depthZ})`;
    }
}

const resize = () => {
    renderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
    camera.aspect = canvas.clientWidth / canvas.clientHeight;
    camera.updateProjectionMatrix();
};
window.addEventListener('resize', resize);

const animate = () => {
    statsRenderer.update();
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
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
