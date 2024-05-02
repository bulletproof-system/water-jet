<template>
  <TresCanvas ref="canvas" preset="realistic">
    <TresPerspectiveCamera 
      ref="camera"
      :fov="45"
      :aspect="1"
      :near="0.1"
      :far="1000"  
      :position="[-5, -5, 10]"
      :rotation="[0, 0, 0]"
      :up="new THREE.Vector3(0, 0, 1)"
    />
    <!-- 相机控制器 -->
    <CameraControl :robot="robot?.robot"/>
    <!-- 坐标系 -->
    <TresGridHelper :position="[0, 0, 0]" :rotation="[Math.PI / 2, 0, 0]" />
    <!-- 光源 -->
    <TresAmbientLight :intensity="0.5" />
    <!-- 地图 -->
    <GlobalMap ref="globalMap"/>
    <!-- 机器人 -->
    <Robot ref="robot" />
    <!-- 激光雷达 -->
    <Lidar ref="laserscan"/>
    <!-- 导航 -->
    <Navigation ref="navigation"/>
    <!-- 花盆 -->
    <!-- <Flowerpots /> -->
  </TresCanvas>
</template>

<script setup lang="ts">
import * as THREE from 'three';
import { TresCanvas } from '@tresjs/core'
import { useMapStore, MouseAction } from '@/stores/map';

const mapStore = useMapStore();
const { mouseAction, arrow } = storeToRefs(mapStore);
const canvas = ref(null)
const camera = ref(null)
const globalMap = ref(null)
const robot = ref(null)
const navigation = ref(null)

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();

let element: Element;

const activeAction: Record<MouseAction, {}> = {
  [MouseAction.Control]: {
    'click': handleClick
  },
  [MouseAction.Navigate]: {
    'mousedown': handleMouseDown,
    'mousemove' : handleMouseMove,
    'mouseup': handleMouseUp,
  },
  [MouseAction.SetPosition]: {
    'mousedown': handleMouseDown,
    'mousemove' : handleMouseMove,
    'mouseup': handleMouseUp,
  }
}

onMounted(() => {
  element = canvas.value.context.renderer.value.domElement
  element.addEventListener('click', handleClick, false);
  watch(mouseAction, (newAction, oldAction) => {
    // 切换鼠标操作后, 移除旧操作的监听器
    for (const event in activeAction[oldAction])
      element.removeEventListener(event, activeAction[oldAction][event]);

    // 添加新操作的监听器
    for (const event in activeAction[newAction])
      element.addEventListener(event, activeAction[newAction][event]);
  })
})

onUnmounted(() => {
  element.removeEventListener('click', handleClick, false);
})

function setRaycaster(clientX: number, clientY: number) {
  const dom = canvas.value.context.renderer.value.domElement
  pointer.x = ( (clientX - dom.getBoundingClientRect().left) / dom.offsetWidth) * 2 - 1;
  pointer.y = - ( (clientY - dom.getBoundingClientRect().top) / dom.offsetHeight ) * 2 + 1;

  raycaster.setFromCamera(pointer, camera.value);
}

function handleClick(event) {
  setRaycaster(event.clientX, event.clientY);
  
}

function handleMouseDown(event) {
  setRaycaster(event.clientX, event.clientY);

  // globalMap 记录箭头起点
  globalMap.value.handleMouseDown(raycaster);
}

function handleMouseMove(event) {
  setRaycaster(event.clientX, event.clientY);

  // globalMap 记录箭头终点
  globalMap.value.handleMouseMove(raycaster);
}

function handleMouseUp(event) {
  switch (mouseAction.value) {
    case MouseAction.Control:
      break;
    case MouseAction.Navigate:
      // 松开后调用导航
      navigation.value.navigate();
      globalMap.value.handleMouseUp();
      mouseAction.value = MouseAction.Control;
      break;
    case MouseAction.SetPosition:
      // 松开后设置机器人位置
      robot.value.setPosition();
      globalMap.value.handleMouseUp();
      mouseAction.value = MouseAction.Control;
      break;
  }
}


</script>