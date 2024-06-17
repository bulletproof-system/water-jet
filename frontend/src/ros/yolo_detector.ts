import * as ROSLIB from 'roslib';
import { Msg, Srv, ros } from '.';
import * as THREE from 'three';
import { useMapStore } from '@/stores/map';
import { useAppStore } from '@/stores/app';

const mapStore = useMapStore();
const appStore = useAppStore();
const { enableObjectDetect, yoloImage } = storeToRefs(mapStore);

let yoloImageListener = new ROSLIB.Topic({
    ros: ros,
	name: '/yolo_detector/detection_image',
	messageType: 'sensor_msgs/Image',
	throttle_rate: 10,
})

let objectDetectStateListener = new ROSLIB.Topic({
    ros: ros,
	name: '/yolo_detector/state',
	messageType: 'std_msgs/String.msg',
})

let objectDetectService = new ROSLIB.Service({
    ros: ros,
	name: '/yolo_detector/ctrl',
	serviceType: 'std_msgs/String.msg',
})

yoloImageListener.subscribe(procYoloImage)
objectDetectStateListener.subscribe(procObjectDetectState)

function procYoloImage(message: unknown) {
	let image = message as unknown as Msg.sensor.Image;
	let picture = new Uint8Array(image.data.length); 
	atob(image.data).split('').forEach((c, i) => picture[i] = c.charCodeAt(0))
	const blob = new Blob([picture], { type: image.encoding });
	yoloImage.value = URL.createObjectURL(blob);
}



function procObjectDetectState(message: unknown) {
	let state = message as unknown as Msg.String;
	
	switch (state.data) {
	    case "start":
			enableObjectDetect.value = true;
			break;
		case "stop":
			enableObjectDetect.value = false;
			break;
		default:
			console.log("Unknown state: " + state.data);
	}
}

function switchYolo() {
	if (appStore.debug) {
		return new Promise((resolve, reject) => {
			setTimeout(() => {
				enableObjectDetect.value = !enableObjectDetect.value;
				resolve(null);
			}, 1000)
		})
	}
}

export {
	switchYolo
}
