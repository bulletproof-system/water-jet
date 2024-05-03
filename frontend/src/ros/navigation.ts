import { ros, Msg } from '@/ros'
import * as ROSLIB from 'roslib';
import * as THREE from 'three';
import { useTresContext } from '@tresjs/core';
import GlobalMap from './GlobalMap.vue';
import { ShallowRef } from 'vue';
import { useAppStore } from '@/stores/app';

// const { scene } = useTresContext()
const appStore = useAppStore()

let navigationPathListener = new ROSLIB.Topic({
	ros: ros,
	name: '/move_base/GlobalPlanner/plan',
	messageType: 'nav_msgs/Path',
	throttle_rate: 100,
})
let navigationAction = {
	publish: new ROSLIB.Topic({
		ros: ros,
		name: '/usr_nav_goal',
		messageType: 'std_msgs/String',
		throttle_rate: 100,
	}),
	subscribe: new ROSLIB.Topic({
		ros: ros,
		name: '/usr_nav_result',
		messageType: 'std_msgs/String',
		throttle_rate: 100,
	})
}
let pathMsg: Msg.nav.Path
const geometry = new THREE.BufferGeometry()
const material = new THREE.LineBasicMaterial({ color: 0xff0000, linewidth: 2, });
let navigationPath = new THREE.Line(geometry, material)

navigationPathListener.subscribe(procPathMsg)
navigationAction.subscribe.subscribe(procNavigationResult)

function procPathMsg(message: unknown) {
	pathMsg = message as unknown as Msg.nav.Path
	const point = []
	for (let i = 0; i < pathMsg.poses.length; i++) {
		const position = pathMsg.poses[i].pose.position
		point.push(new THREE.Vector3(position.x, position.y, position.z))
	}
	geometry.setFromPoints(point)
}

// function handleClick(raycaster: THREE.Raycaster, map: ShallowRef<InstanceType<typeof GlobalMap>>) {
// 	const mapGrid = map.value.getMapGrid()
// 	const intersects = raycaster.intersectObject(mapGrid, true)
//     for (let i = 0; i < intersects.length; i++) {
// 		if (intersects[i].object === mapGrid) {
// 			const goal = intersects[i].point
// 			navigate(goal)
// 			return
// 		}
// 	}
// }


function procNavigationResult(message: unknown) {
	const restult = message as unknown as string
	console.log(restult)
}


export { navigationPath }