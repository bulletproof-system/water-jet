<template>
  <TresCanvas ref="canvas" preset="realistic">
    <TresPerspectiveCamera 
      :fov="45"
      :aspect="1"
      :near="0.1"
      :far="1000"  
      :position="[-5, -5, 10]"
      :rotation="[0, 0, 0]"
      :up="new THREE.Vector3(0, 0, 1)"
    />
    <!-- <CameraControls ref="cameraControls" @start="manualControl()" @end="startAutoControlTimer()"/> -->
    <TresGridHelper :position="[0, 0, 0]" :rotation="[Math.PI / 2, 0, 0]" />
    <TresAmbientLight :intensity="0.5" />
    <primitive :object="map" />
  </TresCanvas>
</template>

<script setup lang="ts">
import { ros } from '@/ros'
import * as ROSLIB from 'roslib';
import { OccupancyGridMsg, LaserScanMsg, TransformMsg, MapMetaDateMsg } from '@/ros/msg'
import * as THREE from 'three';
import { TresCanvas, useRenderLoop } from '@tresjs/core'
import { CameraControls  } from '@tresjs/cientos'
import URDFLoader, { URDFRobot } from 'urdf-loader';
import * as TWEEN from '@tweenjs/tween.js'
import { MapControls } from 'three/examples/jsm/controls/MapControls.js';
let mapTf = new ROSLIB.TFClient({
  ros : ros,
  fixedFrame : 'map',
  angularThres : 0.01,
  transThres : 0.01,
  rate : 10.0,
});
let mapListener = new ROSLIB.Topic<OccupancyGridMsg>({
  ros : ros,
  name : '/map',
  messageType : 'nav_msgs/OccupancyGrid',
  throttle_rate: 1000,
  compression: 'cbor'
});
let laserScanListener = new ROSLIB.Topic<LaserScanMsg>({
  ros : ros,
  name : '/scan',
  messageType : 'sensor_msgs/LaserScan',
  throttle_rate: 100,
  compression: 'cbor'
})
let robotDescriptionParam = new ROSLIB.Param({
  ros : ros,
  name : '/robot_description',
})
const canvas = ref(null)
let camera = null
let cameraControls = null

const autoControlDelta = new THREE.Vector3(5, 0, 5)
let autoControlFlag = false;
let autoControlTimer = null;
const autoControlTimeout = 5000
let autoControlImpl = null
let autoControlTarget = {
  x1: 0,
  y1: 0,
  z1: 0,
  x2: 0,
  y2: 0,
  z2: 0,
}


onMounted(() => {
  mapTf.subscribe('laser', procLaserTF);
  mapTf.subscribe('base_footprint', procRobotTF);
  mapListener.subscribe(procMapMsg);
  laserScanListener.subscribe(procLaserScanMsg);
  robotDescriptionParam.get(procRobotDescriptionParam);
  startAutoControlTimer()
  // console.log(canvas)
  // console.log(camera)
  console.log(canvas.value.context)
  camera = canvas.value.context.camera
  cameraControls = new MapControls(canvas.value.context.camera.value, canvas.value.context.renderer.value.domElement)
  cameraControls.addEventListener('start', manualControl)
  cameraControls.addEventListener('end', startAutoControlTimer)
  console.log(cameraControls)
})

onUnmounted(() => {
  mapTf.unsubscribe('laser', procLaserTF);
  mapListener.unsubscribe(procMapMsg);
  laserScanListener.unsubscribe(procLaserScanMsg);
  manualControl()
})

let mapInfo = {
  width: 1,
  height: 1,
  resolution: 1,
  origin: {
    position: new THREE.Vector3(),
    orientation: new THREE.Quaternion()
  }
}
let map = new THREE.Group()
let occupancyGrid: OccupancyGridMsg = null
let color = new Uint8Array(1 * 1 * 4)
let mapTexture: THREE.DataTexture = null
let mapGeometry = new THREE.PlaneGeometry(10, 1)
let mapMaterial = new THREE.MeshBasicMaterial({ side: THREE.DoubleSide, map: mapTexture })
let mapGrid = new THREE.Mesh(mapGeometry, mapMaterial)
map.add(mapGrid)

function procMapMsg (message) {
  function syncMapMetaData() {
    const info = occupancyGrid.info
    if (info.width != mapInfo.width || 
      info.height != mapInfo.height ||
      info.resolution != mapInfo.resolution ||
      info.origin.position.x != mapInfo.origin.position.x ||
      info.origin.position.y != mapInfo.origin.position.y ||
      info.origin.position.z != mapInfo.origin.position.z ||
      info.origin.orientation.x != mapInfo.origin.orientation.x ||
      info.origin.orientation.y != mapInfo.origin.orientation.y ||
      info.origin.orientation.z != mapInfo.origin.orientation.z ||
      info.origin.orientation.w != mapInfo.origin.orientation.w) {
        mapInfo.width = info.width
        mapInfo.height = info.height
        mapInfo.resolution = info.resolution
        mapInfo.origin.position = new THREE.Vector3(info.origin.position.x, info.origin.position.y, info.origin.position.z)
        mapInfo.origin.orientation = new THREE.Quaternion(info.origin.orientation.x, info.origin.orientation.y, info.origin.orientation.z, info.origin.orientation.w)
        return true
      }
    else return false
  }

  occupancyGrid = message as unknown as OccupancyGridMsg;
  if (syncMapMetaData()) {
    map.remove(mapGrid)
    color = new Uint8Array(mapInfo.width * mapInfo.height * 4);
    mapGeometry = new THREE.PlaneGeometry(mapInfo.width, mapInfo.height);
    mapTexture = new THREE.DataTexture(color, mapInfo.width, mapInfo.height, THREE.RGBAFormat);
    mapMaterial = new THREE.MeshBasicMaterial({ side: THREE.DoubleSide, map: mapTexture })
    mapGrid = new THREE.Mesh(mapGeometry, mapMaterial)
    const offset = new THREE.Vector3(mapInfo.width / 2, mapInfo.height / 2, 0).multiplyScalar(mapInfo.resolution).add(mapInfo.origin.position);
    console.log(mapInfo)
    mapGrid.applyMatrix4(new THREE.Matrix4().compose(offset, mapInfo.origin.orientation, new THREE.Vector3(mapInfo.resolution, mapInfo.resolution, 1)))
    map.add(mapGrid)
  }
  let id = 0
  for (let i = 0; i < mapInfo.width; ++i) {
    for (let j = 0; j < mapInfo.height; ++j, ++id) {
      const c = getColor(occupancyGrid.data[id])
      color[id * 4] = c[0];
      color[id * 4 + 1] = c[1];
      color[id * 4 + 2] = c[2];
      color[id * 4 + 3] = c[3];
    }
  }
  mapTexture.needsUpdate = true;
}

let laser = new THREE.Group()
const rainbowGeometry = new THREE.BoxGeometry(0.05, 0.05, 0.05)
const rainbowMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000, side: THREE.FrontSide })
let scanRainbow = new THREE.InstancedMesh(rainbowGeometry, rainbowMaterial, 360)
laser.add(scanRainbow)
map.add(laser)

function procLaserTF(message: unknown) {
  let laserTF = message as unknown as TransformMsg
  laser.position.set(laserTF.translation.x, laserTF.translation.y, laserTF.translation.z)
  const quat = new THREE.Quaternion(laserTF.rotation.x, laserTF.rotation.y, laserTF.rotation.z, laserTF.rotation.w)
  laser.setRotationFromQuaternion(quat)
}

function procRobotTF(message: unknown) {
  if (robot == null) return
  let robotTF = message as unknown as TransformMsg
  robot.position.set(robotTF.translation.x, robotTF.translation.y, robotTF.translation.z)
  const quat = new THREE.Quaternion(robotTF.rotation.x, robotTF.rotation.y, robotTF.rotation.z, robotTF.rotation.w)
  robot.setRotationFromQuaternion(quat)
}

let scan: LaserScanMsg = null

function procLaserScanMsg(message: unknown) {
  scan = message as unknown as LaserScanMsg;
  scanRainbow.instanceMatrix.needsUpdate = false
  for (let i = 0; i < scan.ranges.length; i++) {
    const angle = scan.angle_min + i * scan.angle_increment
    const dist = scan.ranges[i]
    const x = dist * Math.cos(angle)
    const y = dist * Math.sin(angle)
    const z = 0
    scanRainbow.setMatrixAt(i, new THREE.Matrix4().setPosition(new THREE.Vector3(x, y, z)))
  }
  scanRainbow.instanceMatrix.needsUpdate = true
}

let robot: URDFRobot = null;

function procRobotDescriptionParam(message: unknown) {
  const urdfLoader = new URDFLoader(new THREE.LoadingManager());
  urdfLoader.packages = {
    "wpr_simulation": "/wpr_simulation"
  }
  robot = urdfLoader.parse(message as unknown as string);
  map.add(robot)
}

const { onLoop } = useRenderLoop()
onLoop(autoControl)

function manualControl() {
  autoControlFlag = false
  clearTimeout(autoControlTimer)
  console.log("manual control start")
}
function autoControlTargetUpdate() {
  const target = robot.localToWorld(autoControlDelta)
  const lookAt = robot.position
  autoControlTarget = {
    x1: target.x,
    y1: target.y,
    z1: target.z,
    x2: lookAt.x,
    y2: lookAt.y,
    z2: lookAt.z
  }
}
function startAutoControlTimer() {
  autoControlTimer = setTimeout(() => {
    console.log("auto control start")
    autoControlTargetUpdate()
    autoControlImpl = new TWEEN.Tween({
      x1: camera.value.position.x,
      y1: camera.value.position.y,
      z1: camera.value.position.z,
      x2: cameraControls.target.x,
      y2: cameraControls.target.y,
      z2: cameraControls.target.z,
    }).dynamic(true).to(autoControlTarget, 3000).easing(TWEEN.Easing.Quadratic.Out)
    .onUpdate((o) => {
      let { x1, y1, z1, x2, y2, z2 } = o 
      camera.value.position.set(x1, y1, z1)
      cameraControls.target.set(x2, y2, z2)
      cameraControls.update()
    }).start(undefined, true)
    autoControlFlag = true
  }, autoControlTimeout)
}

function autoControl() {
  if (!autoControlFlag) {
    if (autoControlImpl)
      autoControlImpl.stop()
    return
  }
  autoControlTargetUpdate()
  if (!autoControlImpl.isPlaying()) {
    let { x1, y1, z1, x2, y2, z2 } = autoControlTarget
    console.log(robot.quaternion)
    console.log(autoControlTarget)
    // camera.value.position.set(x1, y1, z1)
    // cameraControls.target.set(x2, y2, z2)
    // cameraControls.update()
    return
  }
  autoControlImpl.update()
  cameraControls.update()
}

function hslToRgba(h: number, s: number, l: number) {
    h /= 360;
    s /= 100;
    l /= 100;

    let r: number, g: number, b: number;

    if (s === 0) {
        r = g = b = l; // achromatic
    } else {
        const hue2rgb = (p: number, q: number, t: number) => {
            if (t < 0) t += 1;
            if (t > 1) t -= 1;
            if (t < 1 / 6) return p + (q - p) * 6 * t;
            if (t < 1 / 2) return q;
            if (t < 2 / 3) return p + (q - p) * (2 / 3 - t) * 6;
            return p;
        };

        const q = l < 0.5 ? l * (1 + s) : l + s - l * s;
        const p = 2 * l - q;
        r = hue2rgb(p, q, h + 1 / 3);
        g = hue2rgb(p, q, h);
        b = hue2rgb(p, q, h - 1 / 3);
    }

    return [r * 256, g * 256, b * 256, 255].map((c) => Math.round(c));
}

function getColor(lightness: number) {
	if (lightness < 0) return [255, 255, 255, 0]
	return hslToRgba(180, 100, 40 - 0.4 * lightness);
}
</script>