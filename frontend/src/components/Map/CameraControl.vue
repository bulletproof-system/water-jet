<template>

</template>

<script setup lang="ts">
import * as THREE from 'three';
import { useRenderLoop, useTresContext } from '@tresjs/core';
import { MapControls } from 'three/examples/jsm/controls/MapControls.js';
import * as TWEEN from '@tweenjs/tween.js'
import { useAppStore } from '@/stores/app';
import { useMapStore, MouseAction, ControlMode, Target } from '@/stores/map';
import { robot } from '@/ros';

const { camera, renderer } = useTresContext()
const appStore = useAppStore()
const mapStore = useMapStore()

const { mouseAction, control, mode } = storeToRefs(mapStore);

const cameraControls = new MapControls(camera.value, renderer.value.domElement)
const offset = new THREE.Vector3(5, 0, 5) // 视角偏移
let potRotate = { rotate: 0 } // 视角偏移旋转量
let timer = new Timeout(0);
let tween: TWEEN.Tween<Target> = null;
const followRobotTimeout = 5000
let target: Target = {}
let rotate = new TWEEN.Tween(potRotate).to({
  rotate: Math.PI * 2
}, 15000).easing(TWEEN.Easing.Linear.None).repeat(Infinity);

watch(mouseAction, (newValue) => {
  cameraControls.enabled = newValue === MouseAction.Control
  if (newValue !== MouseAction.Control) {
    mode.value = ControlMode.Normal
  }
}, { immediate: true })

watch(mode, (newValue, oldValue) => {
  if (newValue === oldValue) return;
  timer.clear();
  switch (newValue) {
    case ControlMode.Normal: // 正常模式, 由用户改变视角
      console.log("正常控制")
      rotate.stop();
      break;
    case ControlMode.FollowRobot: // 视角跟随机器人
      console.log("视角跟随机器人")
      rotate.stop();
      break;
    case ControlMode.LocatePot:   // 视角围绕花盆
      console.log("视角围绕花盆")
      rotate.start()
      break;
  }
}, { immediate: true })

// 仅当 MouseAction.Control 时可用


const { onLoop } = useRenderLoop()
onLoop(() => {
  if (!cameraControls.enabled) return;
  // 根据模式决定视角变化方式
  switch (mode.value) {
    case ControlMode.Normal: // 正常模式, 由用户改变视角
      break;
    case ControlMode.FollowRobot: // 视角跟随机器人
      if (!timer.cleared) break;
      updateFollowRobotTarget();
      break;
    case ControlMode.LocatePot:   // 视角围绕花盆
      updateLocatePotTarget();
      rotate.update();
      break;
  }
})

function updateFollowRobotTarget() {
  target.from = robot.position.clone().add(offset);
  target.to = robot.position.clone();
  updateTween();
}

function updateLocatePotTarget() {
  target.from = control.value.target.to.clone().add(control.value.target.from.clone().applyAxisAngle(new THREE.Vector3(0, 0, 1), potRotate.rotate));
  target.to = control.value.target.to.clone();
  // console.log(target, potRotate.rotate)
  updateTween();
}

function updateTween() {
  if (!tween?.isPlaying()) {
    const delta = camera.value.position.distanceTo(target.from) <= 0.5 ? 100 : 3000;
    tween = new TWEEN.Tween({
      from: camera.value.position,
      to: cameraControls.target,
    })
    .dynamic(true)
    .easing(TWEEN.Easing.Quadratic.Out)
    .onUpdate((o) => {
      camera.value.position.copy(o.from),
      cameraControls.target.copy(o.to)
      cameraControls.update()
    }).to(target, delta)
    .start(undefined, true);
  }
  tween.update();
}

function Timeout(timeout: number) {
  this.cleared = false;
  this.id = setTimeout(() => { this.cleared = true }, timeout);
  this.clear = function () {
    this.cleared = true;
    clearTimeout(this.id);
  };
  this.reset = function (timeout: number) {
    this.cleared = false;
    clearTimeout(this.id);
    this.id = setTimeout(() => { this.cleared = true }, timeout);
  };
}

function handleMouseDown() {
  switch (mode.value) {
    case ControlMode.Normal:
      break;
    case ControlMode.FollowRobot:
      timer.reset(followRobotTimeout);
      tween.stop();
      break;
    case ControlMode.LocatePot:
      mode.value = ControlMode.Normal
      break;
  }
}

defineExpose({
  handleMouseDown
});

// const props = defineProps({
// 	robot: THREE.Object3D
// })

// onMounted(() => {
// 	startAutoControlTimer()
// 	cameraControls.addEventListener('start', manualControl)
//  	cameraControls.addEventListener('end', startAutoControlTimer)
// })

// onUnmounted(() => {
//   manualControl()
// 	cameraControls.removeEventListener('start', manualControl)
// 	cameraControls.removeEventListener('end', startAutoControlTimer)
// })

// watch(() => {return appStore.autoControl}, (newValue) => {
//   if (newValue) {
//     startAutoControlTimer()
//   } else {
//     manualControl()
//   }
// })

// const { onLoop } = useRenderLoop()
// onLoop(autoControl)

// function manualControl() {
//   autoControlFlag = false
//   clearTimeout(autoControlTimer)
//   console.log("manual control start")
// }
// function autoControlTargetUpdate() {
//   const target = props.robot.localToWorld(autoControlDelta.clone())
//   const lookAt = props.robot.position
//   autoControlTarget = {
//     x1: target.x,
//     y1: target.y,
//     z1: target.z,
//     x2: lookAt.x,
//     y2: lookAt.y,
//     z2: lookAt.z
//   }
// }
// function startAutoControlTimer() {
//   if (appStore.autoControl === false) return
//   clearTimeout(autoControlTimer)
//   autoControlTimer = setTimeout(() => {
//     console.log("auto control start")
//     autoControlTargetUpdate()
//     autoControlImpl = new TWEEN.Tween({
//       x1: camera.value.position.x,
//       y1: camera.value.position.y,
//       z1: camera.value.position.z,
//       x2: cameraControls.target.x,
//       y2: cameraControls.target.y,
//       z2: cameraControls.target.z,
//     }).dynamic(true).to(autoControlTarget, 3000).easing(TWEEN.Easing.Quadratic.Out)
//     .onUpdate((o) => {
//       let { x1, y1, z1, x2, y2, z2 } = o 
//       camera.value.position.set(x1, y1, z1)
//       cameraControls.target.set(x2, y2, z2)
//       cameraControls.update()
//     }).start(undefined, true)
//     autoControlFlag = true
//   }, autoControlTimeout)
// }

// function autoControl() {
//   if (!autoControlFlag) {
//     if (autoControlImpl)
//       autoControlImpl.stop()
//     return
//   }
//   autoControlTargetUpdate()
//   if (!autoControlImpl.isPlaying()) {
//     let { x1, y1, z1, x2, y2, z2 } = autoControlTarget
//     camera.value.position.set(x1, y1, z1)
//     cameraControls.target.set(x2, y2, z2)
//     cameraControls.update()
//     return
//   }
//   autoControlImpl.update()
//   cameraControls.update()
// }

</script>