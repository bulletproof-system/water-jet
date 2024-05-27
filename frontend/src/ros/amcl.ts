import * as ROSLIB from 'roslib';
import { Msg, Srv, ros } from '.';
import * as THREE from 'three';
import { useAppStore } from '@/stores/app';

let particlecloudListener = new ROSLIB.Topic({
    ros: ros,
	name: '/particlecloud',
	messageType: 'geometry_msgs/PoseArray',
	throttle_rate: 10,
})

let acmlPoseListener = new ROSLIB.Topic({
    ros: ros,
	name: '/amcl_pose',
	messageType: 'geometry_msgs/PoseWithCovarianceStamped',
	throttle_rate: 10,
})

let particlecloudMsg: Msg.geometry.PoseArray;
let acmlPoseMsg: Msg.geometry.PoseWithCovarianceStamped;

const arrowHelper = new THREE.ArrowHelper(
	new THREE.Vector3(0, 0, 0),
	new THREE.Vector3(0, 0, 0),
	0.2,
	0x00ff00,
	0.05,
	0.03
);

const acmlPoseArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 0, 0),
	new THREE.Vector3(0, 0, 0),
	0.2,
	0xff0000,
	0.05,
	0.05
)

let arrows = new THREE.Group()

particlecloudListener.subscribe(procParticleCloud);
acmlPoseListener.subscribe(procACMLPose);

function procParticleCloud(message: unknown) {
    particlecloudMsg = message as unknown as Msg.geometry.PoseArray;
	console.log(particlecloudMsg)
	// 清除箭头
	arrows.clear();

	// 遍历粒子云中的每个粒子
	for (let i = 0; i < particlecloudMsg.poses.length; i++) {
	    // 获取粒子的位置和方向
	    let pose = particlecloudMsg.poses[i];
	    let position = new THREE.Vector3(pose.position.x, pose.position.y, pose.position.z);
	    
	    // 创建箭头
	    let arrow = arrowHelper.clone();
	    arrow.position.copy(position);
		arrow.setRotationFromQuaternion(new THREE.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
		arrow.rotateOnAxis(new THREE.Vector3(0, 0, 1), -Math.PI / 2) // 修正角度, 不知道为什么偏 90 度
		arrows.add(arrow);
	}
}

function procACMLPose(message: unknown) {
    acmlPoseMsg = message as unknown as Msg.geometry.PoseWithCovarianceStamped;
	const pose = acmlPoseMsg.pose.pose;
	acmlPoseArrow.position.set(pose.position.x, pose.position.y, pose.position.z);
	acmlPoseArrow.setRotationFromQuaternion(new THREE.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
	acmlPoseArrow.rotateOnAxis(new THREE.Vector3(0, 0, 1), -Math.PI / 2) // 修正角度, 不知道为什么偏 90 度
}

export {
	arrows,
	acmlPoseArrow
}