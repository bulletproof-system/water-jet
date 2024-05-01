import { ros, TF, Msg } from '@/ros'
import * as ROSLIB from 'roslib';
import * as THREE from 'three';

let laserScanListener = new ROSLIB.Topic<Msg.sensor.LaserScan>({
	ros: ros,
	name: '/scan',
	messageType: 'sensor_msgs/LaserScan',
	throttle_rate: 100,
	compression: 'cbor'
})
let laser = new THREE.Group()
const rainbowGeometry = new THREE.BoxGeometry(0.05, 0.05, 0.05)
const rainbowMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000, side: THREE.FrontSide })
let scanRainbow = new THREE.InstancedMesh(rainbowGeometry, rainbowMaterial, 360)
laser.add(scanRainbow)
let laserScanMsg: Msg.sensor.LaserScan = null

TF.map.subscribe('laser', procLaserTF);
laserScanListener.subscribe(procLaserScanMsg);

function procLaserTF(message: unknown) {
	let laserTF = message as unknown as Msg.geometry.Transform
	laser.position.set(laserTF.translation.x, laserTF.translation.y, laserTF.translation.z)
	const quat = new THREE.Quaternion(laserTF.rotation.x, laserTF.rotation.y, laserTF.rotation.z, laserTF.rotation.w)
	laser.setRotationFromQuaternion(quat)
}

function procLaserScanMsg(message: unknown) {
	laserScanMsg = message as unknown as Msg.sensor.LaserScan;
	scanRainbow.instanceMatrix.needsUpdate = false
	for (let i = 0; i < laserScanMsg.ranges.length; i++) {
		const angle = laserScanMsg.angle_min + i * laserScanMsg.angle_increment
		const dist = laserScanMsg.ranges[i]
		const x = dist * Math.cos(angle)
		const y = dist * Math.sin(angle)
		const z = 0
		scanRainbow.setMatrixAt(i, new THREE.Matrix4().setPosition(new THREE.Vector3(x, y, z)))
	}
	scanRainbow.instanceMatrix.needsUpdate = true
}

export { laser }