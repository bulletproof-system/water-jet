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
  tween?.stop();
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
  target.from = robot.localToWorld(offset.clone());
  target.to = robot.position.clone();
  updateTween();
}

function updateLocatePotTarget() {
  target.from = control.value.target.to.clone().add(control.value.target.from.clone().applyAxisAngle(new THREE.Vector3(0, 0, 1), potRotate.rotate));
  target.to = control.value.target.to.clone();
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

</script>