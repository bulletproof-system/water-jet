<template>
	<primitive :object="globalMap"/>
</template>

<script setup lang="ts">
import * as THREE from 'three'
import { globalMap } from '@/ros'
import { useMapStore } from '@/stores/map';

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

defineExpose({
	handleMouseDown,
	handleMouseMove,
	handleMouseUp,
})

</script>