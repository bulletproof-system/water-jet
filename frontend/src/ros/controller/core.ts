import * as ROSLIB from 'roslib';
import { ros } from "../init";
import { Msg, Srv } from "../";
import { CtrlMode, NodeState, useROSStore } from "@/stores/ros";
import { useAppStore } from '@/stores/app';
import { error } from 'console';

// 订阅 info, 更新 controller 状态
const infoTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/ctrl/info",
    messageType: "controller/Info"
})
const appStore = useAppStore();
const rosStore = useROSStore();
const { ctrlMode, scram, nodeInfo } = storeToRefs(rosStore);

infoTopic.subscribe((message: ROSLIB.Message) => {
	const info = message as unknown as Msg.controller.Info;
	rosStore.setCtrlMode(info.mode);
	rosStore.setScram(info.scram);
})

// 切换 mode
const changeModeService = new ROSLIB.Service({
	ros: ros,
	name: '/ctrl/change_mode',
	serviceType: 'controller/ChangeMode'
});
export function changeMode(mode: CtrlMode): Promise<any> {
    if (ctrlMode.value === mode) return Promise.resolve(undefined);
	const request: Srv.controller.ChangeMode.Request = {
		mode: mode
	};
	return new Promise((resolve, reject) => {
		changeModeService.callService(request, (response: Srv.controller.ChangeMode.Response) => {
			if (response.success) {
				resolve(response);
			} else {
				console.warn('change mode failed');
				reject("change mode failed");
			}
		}, (error) => {
			console.warn(error);
			reject(error);
		})
	});
}

// 切换急停
const scramServive = new ROSLIB.Service({
    ros: ros,
    name: '/ctrl/scram',
    serviceType: 'controller/Scram'
})
export function toggleScram(active: boolean): Promise<any> {
    if (scram.value === active) return;
	const request: Srv.controller.Scram.Request = {
		active: active
	};
	return new Promise((resolve, reject) => {
		scramServive.callService(request, (response: Srv.controller.Scram.Response) => {
			if (response.success) {
				resolve(response);
			} else {
				console.warn('toggle scram failed');
				reject("toggle scram failed");
			}
		}, (error) => {
			console.warn(error);
			reject(error);
		})

		// debug 时模拟切换
		if (appStore.debug) {
			setTimeout(() => {
				rosStore.setScram(active)
			}, 1000);
		}
	});
	
}

// 启动节点
const startService = new ROSLIB.Service({
    ros: ros,
    name: '/ctrl/start',
    serviceType: 'controller/Start'
});
export function start(): Promise<any> {
    if (nodeInfo.value.state !== NodeState.Stop) {
		return Promise.reject(`node can't start: ${nodeInfo}`);
	}
	nodeInfo.value = {
		state: NodeState.Stop,
		task: '正在启动节点',
		feedback: '',
		result: '',
		percentage: -1,
		cancel: null
	}
	const request: Srv.controller.Start.Request = {
		mode: ctrlMode.value
	};
	return new Promise((resolve, reject) => {
		startService.callService(request, (response: Srv.controller.Start.Response) => {
			if (response.success) {
				nodeInfo.value = {
					state: NodeState.Wait,
					task: '',
					feedback: '启动节点完成',
					result: 'success',
					percentage: 100,
					cancel: null
				}
				resolve(response);
			} else {
				nodeInfo.value = {
					state: NodeState.Stop,
					task: '',
					feedback: '启动节点失败',
					result: 'fail',
					percentage: 100,
					cancel: null
				}
				console.warn('start node failed');
				reject("start node failed");
			}
		}, (error) => {
			nodeInfo.value = {
				state: NodeState.Stop,
				task: '',
				feedback: '启动节点失败',
				result: 'error',
				percentage: 100,
				cancel: null
			}
			console.warn(error);
			reject(error);
		})
	})
	
};

// 停止节点
const stopService = new ROSLIB.Service({
    ros: ros,
    name: '/ctrl/stop',
    serviceType: 'controller/Stop'
});
export function stop(): Promise<any> {
	if (nodeInfo.value.state === NodeState.Stop) {
		console.error("node already stop: ", nodeInfo);
		return Promise.reject(`node already stop: ${nodeInfo}`);
	}
	// 停止正在运行的任务
	rosStore.cancel();
	nodeInfo.value = {
		state: NodeState.Wait,
		task: '正在停止节点',
		feedback: '',
		result: '',
		percentage: -1,
		cancel: null
	}
	const request: Srv.controller.Stop.Request = {
		mode: ctrlMode.value
	};
	return new Promise((resolve, reject) => {
		stopService.callService(request, (response: Srv.controller.Stop.Response) => {
			if (response.success) {
				nodeInfo.value = {
					state: NodeState.Stop,
					task: '',
					feedback: '停止节点成功',
					result: 'success',
					percentage: 100,
					cancel: null
				}
				resolve(response);
			} else {
				nodeInfo.value = {
					state: NodeState.Stop,
					task: '',
					feedback: '停止节点失败',
					result: 'fail',
					percentage: 100,
					cancel: null
				}
				console.warn('stop node failed');
				reject("stop node failed");
			}
		}, (error) => {
			nodeInfo.value = {
				state: NodeState.Stop,
				task: '',
				feedback: '停止节点失败',
				result: 'error',
				percentage: 100,
				cancel: null
			}
			console.warn(error);
			reject(error);
		});
	});
	
}