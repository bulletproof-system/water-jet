<template>
	<primitive :object="navigationPath" />
</template>

<script setup lang="ts">
import { ros } from '@/ros'
import * as ROSLIB from 'roslib';
import { PathMsg } from '@/ros/msg'
import * as THREE from 'three';
import { useTresContext } from '@tresjs/core';

const { scene } = useTresContext()

let navigationPathListener = new ROSLIB.Topic({
    ros: ros,
	name: '/move_base/GlobalPlanner/plan',
	messageType: 'nav_msgs/Path',
	throttle_rate: 100,
})
let pathMsg: PathMsg
const geometry = new THREE.BufferGeometry()
const material = new THREE.LineBasicMaterial( { color: 0xff0000, linewidth: 2,  } );
let navigationPath = new THREE.Line(geometry, material)

defineExpose({
	navigationPath
})

onMounted(() => {
	navigationPathListener.subscribe(procPathMsg)
})

onUnmounted(() => {
	navigationPathListener.unsubscribe(procPathMsg)
})

function procPathMsg(message: unknown) {
    pathMsg = message as unknown as PathMsg
	const point = []
	for (let i = 0; i < pathMsg.poses.length; i++) {
		const position = pathMsg.poses[i].pose.position
		point.push(new THREE.Vector3(position.x, position.y, position.z))
	}
	geometry.setFromPoints(point)
}

</script>