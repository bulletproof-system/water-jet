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
    <CameraControl ref="cameraControl"/>
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
    <Flowerpots ref="flowerPots" />
  </TresCanvas>
  <div style="position: relative;" class="d-flex justify-end">
    <div style="position: absolute; top: -190px" class="d-flex flex-column pa-3">
        <v-btn  class="ma-1" icon
          @click="mode = MapControlMode.Normal"
        >
          <v-icon> {{ mode === MapControlMode.Normal ? 'mdi-map-check-outline' : 'mdi-map-outline' }} </v-icon>
          <v-tooltip activator="parent" location="start">
            自由视角
          </v-tooltip>
        </v-btn>
        <v-btn  class="ma-1" icon
          @click="mode = MapControlMode.FollowRobot"
        >
          <v-icon> {{ mode === MapControlMode.FollowRobot ? 'mdi-robot-happy' : 'mdi-robot' }} </v-icon>
          <v-tooltip activator="parent" location="start" >
            跟随机器人
          </v-tooltip>
        </v-btn>
        <v-btn  class="ma-1" icon
          @click="enbaleSelectPot = !enbaleSelectPot"
        >
          <v-icon> {{ enbaleSelectPot ? 'mdi-flower-tulip' : 'mdi-flower-tulip-outline' }} </v-icon>
          <v-tooltip activator="parent" location="start" >
            选择花盆
          </v-tooltip>
        </v-btn>
    </div>
    
  </div>

</template>

<script setup lang="ts">
import * as THREE from 'three';
import { TresCanvas } from '@tresjs/core'
import { useMapStore, MouseAction, ControlMode as MapControlMode } from '@/stores/map';

const mapStore = useMapStore();
const { mouseAction, mode } = storeToRefs(mapStore);
const cameraControl = ref(null)
const canvas = ref(null)
const camera = ref(null)
const globalMap = ref(null)
const robot = ref(null)
const navigation = ref(null)
const flowerPots = ref(null)
const enbaleSelectPot = ref(false)

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();

let element: Element;

const activeAction: Record<MouseAction, {}> = {
  [MouseAction.Control]: {
    'click': handleClick,
    'mousedown': handleMouseDown,
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
  watch(mouseAction, (newAction, oldAction) => {
    // 切换鼠标操作后, 移除旧操作的监听器
    for (const event in activeAction[oldAction])
      element.removeEventListener(event, activeAction[oldAction][event]);

    // 添加新操作的监听器
    for (const event in activeAction[newAction])
      element.addEventListener(event, activeAction[newAction][event]);
  }, { immediate: true })
})

onUnmounted(() => {

})

function setRaycaster(clientX: number, clientY: number) {
  const dom = canvas.value.context.renderer.value.domElement
  pointer.x = ( (clientX - dom.getBoundingClientRect().left) / dom.offsetWidth) * 2 - 1;
  pointer.y = - ( (clientY - dom.getBoundingClientRect().top) / dom.offsetHeight ) * 2 + 1;

  raycaster.setFromCamera(pointer, camera.value);
}

function handleClick(event) {
  setRaycaster(event.clientX, event.clientY);
  
  if (enbaleSelectPot.value)
    flowerPots.value.handleClick(raycaster);
}

function handleMouseDown(event) {
  setRaycaster(event.clientX, event.clientY);
  switch (mouseAction.value) {
    case MouseAction.Control:
      cameraControl.value.handleMouseDown();  
      break;
    case MouseAction.Navigate:
      // globalMap 记录箭头起点
      globalMap.value.handleMouseDown(raycaster);
      break;
    case MouseAction.SetPosition:
      // globalMap 记录箭头起点
      globalMap.value.handleMouseDown(raycaster);
      break;
  }
  
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