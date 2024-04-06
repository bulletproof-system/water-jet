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
    <CameraControl :robot="robot?.robot"/>
    <TresGridHelper :position="[0, 0, 0]" :rotation="[Math.PI / 2, 0, 0]" />
    <TresAmbientLight :intensity="0.5" />
    <GlobalMap ref="gloalMap"/>
    <Robot ref="robot" />
    <LaserScan ref="laserscan"/>
    <NavigationPath ref="navigationPath"/>
  </TresCanvas>
</template>

<script setup lang="ts">
import * as THREE from 'three';
import { TresCanvas } from '@tresjs/core'

const robot = shallowRef(null)
</script>