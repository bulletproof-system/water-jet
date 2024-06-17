import { ros, Msg } from '@/ros'
import * as ROSLIB from 'roslib';
import * as THREE from 'three';

let navigationPathListener = new ROSLIB.Topic({
	ros: ros,
	name: '/move_base/GlobalPlanner/plan',
	messageType: 'nav_msgs/Path',
	throttle_rate: 100,
})

let pathMsg: Msg.nav.Path
const geometry = new THREE.BufferGeometry()
const material = new THREE.LineBasicMaterial({ color: 0xff0000, linewidth: 2, });
let navigationPath = new THREE.Line(geometry, material)

navigationPathListener.subscribe(procPathMsg)

function procPathMsg(message: unknown) {
	pathMsg = message as unknown as Msg.nav.Path
	const point = []
	for (let i = 0; i < pathMsg.poses.length; i++) {
		const position = pathMsg.poses[i].pose.position
		point.push(new THREE.Vector3(position.x, position.y, position.z))
	}
	geometry.setFromPoints(point)
}

export { navigationPath }