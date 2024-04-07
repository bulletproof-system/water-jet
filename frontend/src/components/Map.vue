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
    <CameraControl :robot="robot?.robot"/>
    <TresGridHelper :position="[0, 0, 0]" :rotation="[Math.PI / 2, 0, 0]" />
    <TresAmbientLight :intensity="0.5" />
    <GlobalMap ref="globalMap"/>
    <Robot ref="robot" />
    <LaserScan ref="laserscan"/>
    <Navigation ref="navigation"/>
    <Flowerpots />
  </TresCanvas>
</template>

<script setup lang="ts">
import * as THREE from 'three';
import { TresCanvas } from '@tresjs/core'
import { handleError } from 'vue';

const canvas = ref(null)
const camera = ref(null)
const globalMap = ref(null)
const robot = ref(null)
const navigation = ref(null)

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();

onMounted(() => {
  const dom = canvas.value.context.renderer.value.domElement
  console.log(canvas.value.context)
  dom.addEventListener('click', onClick, false);
})

onUnmounted(() => {
  document.removeEventListener('click', onClick, false);
})


function onClick(event) {
  const dom = canvas.value.context.renderer.value.domElement
  pointer.x = ( (event.clientX - dom.getBoundingClientRect().left) / dom.offsetWidth) * 2 - 1;
  pointer.y = - ( (event.clientY - dom.getBoundingClientRect().top) / dom.offsetHeight ) * 2 + 1;

  raycaster.setFromCamera(pointer, camera.value);

  navigation.value.handleClick(raycaster, globalMap)
}

</script>