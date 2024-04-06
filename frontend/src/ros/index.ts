
import * as ROSLIB from 'roslib';

import { useROSStore, ROSState } from '@/stores/ros';

const websocket_url = 'ws://localhost:9090';
const retry = 5; // 最多重试次数
const timeout = 5000; // 重试间隔

export const ros = new ROSLIB.Ros({});

const rosStore = useROSStore();

ros.on('connection', () => {
	rosStore.setRosState(ROSState.Connected)
	rosStore.resetRetry()
	console.log('Connected to websocket server.');
})

ros.on('error', (error) => {
	rosStore.setRosState(ROSState.Error)
	console.log('Error connecting to websocket server: ', error);
})

ros.on('close', () => {
	rosStore.setRosState(ROSState.Disconnected)
	console.log('Connection to websocket server closed.');
})

export const connectROS = () => {
	rosStore.setRosState(ROSState.Connecting)
	rosStore.retry++;
	ros.connect(websocket_url);
}

connectROS();

rosStore.$onAction(({ store, after }) => {
	after(() => {
		if (store.state === ROSState.Disconnected) {
			if (store.retry < retry) {
				setTimeout(() => {
					connectROS();
				}, timeout)
			} else store.setRosState(ROSState.Error)
		}
	})
})

export let mapTf = new ROSLIB.TFClient({
	ros : ros,
	fixedFrame : 'map',
	angularThres : 0.01,
	transThres : 0.01,
	rate : 10.0,
});