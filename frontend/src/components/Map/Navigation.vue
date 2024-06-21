<template>
	<primitive :object="navigationPath" />
</template>

<script setup lang="ts">
import * as THREE from 'three'
import { navigationPath, ctrl, Msg } from '@/ros'
import { useMapStore } from '@/stores/map';

const mapStore = useMapStore();
const { arrow } = storeToRefs(mapStore);

function navigate() {
	const quaternion = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(1, 0, 0),  arrow.value.direction.normalize());
	const pos: Msg.geometry.Pose = {
		position: { ...arrow.value.origin, z:0 },
		orientation: { x: quaternion.x, y: quaternion.y, z: quaternion.z , w: quaternion.w }
	}
	ctrl.navigate(pos);
}

defineExpose({
	navigate
});

</script>