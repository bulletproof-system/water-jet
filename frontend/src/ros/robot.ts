import { ros, TF, Msg } from '@/ros'
import * as ROSLIB from 'roslib';
import * as THREE from 'three';
import URDFLoader, { URDFRobot } from 'urdf-loader';

let robotDescriptionParam = new ROSLIB.Param({
	ros : ros,
	name : '/robot_description',
  })
let robot = new THREE.Group()
let robotModel: URDFRobot = null;

TF.map.subscribe('base_footprint', procRobotTF);
robotDescriptionParam.get(procRobotDescriptionParam);

function procRobotTF(message: unknown) {
	if (robot == null) return
	let robotTF = message as unknown as Msg.geometry.Transform
	robot.position.set(robotTF.translation.x, robotTF.translation.y, robotTF.translation.z)
	const quat = new THREE.Quaternion(robotTF.rotation.x, robotTF.rotation.y, robotTF.rotation.z, robotTF.rotation.w)
	robot.setRotationFromQuaternion(quat)
}

function procRobotDescriptionParam(message: unknown) {
	const urdfLoader = new URDFLoader(new THREE.LoadingManager());
	urdfLoader.packages = {
		"wpr_simulation": "/wpr_simulation",
		"wpb_home_bringup": "/wpb_home_bringup"
	}
	robotModel = urdfLoader.parse(message as unknown as string);
	robot.add(robotModel)
}



export { robot }