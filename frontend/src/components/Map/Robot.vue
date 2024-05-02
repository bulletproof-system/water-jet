<template>
	<primitive :object="robot" />
</template>

<script setup lang="ts">
import * as THREE from 'three'
import { robot, ctrl, Msg } from '@/ros'
import { useMapStore } from '@/stores/map';

const mapStore = useMapStore();
const { arrow } = storeToRefs(mapStore);

function setPosition() {
	const pos: Msg.geometry.Pose = {
		position: { ...arrow.value.origin },
		orientation: { ...new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(1, 0, 0),  arrow.value.direction.normalize()) }
	}
	ctrl.manualInitPose(pos);
}

defineExpose({
	setPosition
});

</script>