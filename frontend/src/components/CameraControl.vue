<template>

</template>

<script setup lang="ts">
import * as THREE from 'three';
import { useRenderLoop, useTresContext } from '@tresjs/core';
import { MapControls } from 'three/examples/jsm/controls/MapControls.js';
import * as TWEEN from '@tweenjs/tween.js'

const { camera, renderer } = useTresContext()

const cameraControls = new MapControls(camera.value, renderer.value.domElement)
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

const props = defineProps({
	robot: THREE.Object3D
})

onMounted(() => {
	startAutoControlTimer()
	cameraControls.addEventListener('start', manualControl)
 	cameraControls.addEventListener('end', startAutoControlTimer)
})

onUnmounted(() => {
    manualControl()
	cameraControls.removeEventListener('start', manualControl)
	cameraControls.removeEventListener('end', startAutoControlTimer)
})

const { onLoop } = useRenderLoop()
onLoop(autoControl)

function manualControl() {
  autoControlFlag = false
  clearTimeout(autoControlTimer)
  console.log("manual control start")
}
function autoControlTargetUpdate() {
  const target = props.robot.localToWorld(autoControlDelta.clone())
  const lookAt = props.robot.position
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
    camera.value.position.set(x1, y1, z1)
    cameraControls.target.set(x2, y2, z2)
    cameraControls.update()
    return
  }
  autoControlImpl.update()
  cameraControls.update()
}

</script>