<template>
	<primitive :object="flowerpots" />
</template>


<script lang="ts" setup>
import { ros, mapTf, Msg } from '@/ros'
import * as ROSLIB from 'roslib';
import * as THREE from 'three';
import { useAppStore } from '@/stores/app';

const appStore = useAppStore()
const flowerpots = new THREE.Group();
const kinect2 = new THREE.Group();
const flowerpotListener = new ROSLIB.Topic({
    ros: ros,
	name: '/object_centers',
	messageType: 'geometry_msgs/PointStamped',
	throttle_rate: 500,
  	compression: 'cbor'
})
const flowerpotGeometry = new THREE.CylinderGeometry( 0.05, 0.05, 0.5, 10 ).rotateX( Math.PI / 2 );
const flowerpotMaterial = new THREE.MeshBasicMaterial( { color: 0x0000ff } );

onMounted(() => {
	mapTf.subscribe('base_footprint', procRobotTF)
	flowerpotListener.subscribe(procPointStampedMsg)
})

onUnmounted(() => {
	mapTf.unsubscribe('base_footprint', procRobotTF);
	flowerpotListener.unsubscribe(procPointStampedMsg)
})

function procRobotTF(message: unknown) {
	let robotTF = message as unknown as Msg.Transform
	kinect2.position.set(robotTF.translation.x, robotTF.translation.y, robotTF.translation.z)
	const quat = new THREE.Quaternion(robotTF.rotation.x, robotTF.rotation.y, robotTF.rotation.z, robotTF.rotation.w)
	kinect2.setRotationFromQuaternion(quat)
	// console.log(kinect2.position)
}

function procPointStampedMsg(message: unknown) {
	let pointStamped = message as unknown as Msg.PointStamped
	if (appStore.allowObjectRecognition === false) return
	const point = kinect2.localToWorld(new THREE.Vector3(pointStamped.point.x, pointStamped.point.y, pointStamped.point.z))
	console.log(point)
	for (let i = 0; i < flowerpots.children.length; i++) {
		if (point.distanceTo(flowerpots.children[i].position) < 0.5)
			return
	}
	const flowerpot = new THREE.Mesh(flowerpotGeometry, flowerpotMaterial)
	flowerpot.position.copy(point)
	flowerpots.add(flowerpot)
}


</script>