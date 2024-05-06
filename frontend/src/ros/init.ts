
import * as ROSLIB from 'roslib';
import * as THREE from 'three';
import { Msg } from '.';

import { useROSStore, ConnectState } from '@/stores/ros';

const websocket_url = 'ws://localhost:9090';
const retry = 5; // 最多重试次数
const timeout = 5000; // 重试间隔

export const ros = new ROSLIB.Ros({}); 

const rosStore = useROSStore();

ros.on('connection', () => {
	rosStore.setConnectState(ConnectState.Connected)
	rosStore.resetRetry()
	console.log('Connected to websocket server.');
})

ros.on('error', (error) => {
	rosStore.setConnectState(ConnectState.Error)
	console.log('Error connecting to websocket server: ', error);
})

ros.on('close', () => {
	rosStore.setConnectState(ConnectState.Disconnected)
	console.log('Connection to websocket server closed.');
})

export const connectROS = (reset? : boolean) => {
	if (rosStore.connectState === ConnectState.Connected) return
	if (rosStore.connectState === ConnectState.Connecting) return
	if (reset) rosStore.resetRetry()
	rosStore.setConnectState(ConnectState.Connecting)
	rosStore.retry++;
	ros.connect(websocket_url);
}

connectROS();

rosStore.$onAction(({ store, after }) => {
	after(() => {
		if (store.connectState === ConnectState.Disconnected) {
			if (store.retry < retry) {
				setTimeout(() => {
					connectROS();
				}, timeout)
			} else store.setConnectState(ConnectState.Error)
		}
	})
})


export interface Pose  {
	position: THREE.Vector3,
	quaternion: THREE.Quaternion,
}