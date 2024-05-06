<template>
	<primitive :object="flowerpots" />
</template>


<script lang="ts" setup>
import { flowerpots, pots } from '@/ros'
import { useMapStore } from '@/stores/map';
import { useAppStore } from '@/stores/app';
import * as THREE from 'three';

const appStore = useAppStore();
const mapStore = useMapStore();

const from = new THREE.Vector3(5, 0, 5);

function locatePot(id: string) {
	if (pots.value.has(id)) {
		const pot = pots.value.get(id)
		mapStore.setTarget(from, new THREE.Vector3().copy(pot.pose.position))
	}
}

function handleClick(raycaster: THREE.Raycaster) {
	const intersects = raycaster.intersectObjects(flowerpots.children, true);
	console.log(intersects)
	if (intersects.length > 0) {
		const id = intersects[0].object.userData.id;
		locatePot(id);
		appStore.openPot(id);
	}
}

onMounted(() => {
	mapStore.setLocatePot(locatePot)
});

defineExpose({
	handleClick,
});
</script>