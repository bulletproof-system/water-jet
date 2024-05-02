<template>
	<primitive :object="globalMap"/>
	<primitive v-if="arrow.origin" :object="arrowHelper" />
</template>

<script setup lang="ts">
import * as THREE from 'three'
import { globalMap } from '@/ros'
import { useMapStore } from '@/stores/map';
import { useRenderLoop } from '@tresjs/core';

const mapStore = useMapStore();
const { arrow } = storeToRefs(mapStore);

function handleMouseDown(raycaster: THREE.Raycaster) {
	const interset = raycaster.intersectObject(globalMap);
	if (interset.length == 0) {
		return;
	}
	arrow.value.origin = interset[0].point;
}

function handleMouseMove(raycaster: THREE.Raycaster) {
	const interset = raycaster.intersectObject(globalMap);
	if (interset.length == 0) {
		return;
	}
	if (!arrow.value.origin) {
		return;
	}
	arrow.value.direction = interset[0].point.clone().sub(arrow.value.origin);
}

function handleMouseUp() {
	arrow.value.origin = null;
	arrow.value.direction = null;
}

const arrowHelper = new THREE.ArrowHelper(
	new THREE.Vector3(0, 0, 0),
	new THREE.Vector3(0, 0, 0),
	1,
	0xff0000,
	0.1,
	0.1
);

const { onLoop } = useRenderLoop();
onLoop(() => {
	if (arrow.value.origin && arrow.value.direction) {
		arrowHelper.setDirection(arrow.value.direction.clone().normalize());
		arrowHelper.position.copy(arrow.value.origin);
	}
});


defineExpose({
	handleMouseDown,
	handleMouseMove,
	handleMouseUp,
})

</script>