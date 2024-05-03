import { Srv, Msg, ros, Action } from "..";
import { useROSStore, NodeState } from "@/stores/ros";
import * as ROSLIB from "roslib";

const rosStore = useROSStore();
const { ctrlMode, nodeInfo } = storeToRefs(rosStore)

// 删除地图
const clearMapService = new ROSLIB.Service({
	ros: ros,
	name: '/ctrl/pending/clear_map',
	serviceType: '/controller/ClearMap'
})
function clearMap(): Promise<any> {
	nodeInfo.value = {
		state: NodeState.Clear_Map,
		task: '删除地图',
		feedback: '正在删除...',
		result: '',
		percentage: -1,
		cancel: null
	}
	const requset: Srv.controller.ClearMap.Request = {
	    clear: true
	}
	return new Promise((resolve, reject) => {
		clearMapService.callService(requset, (response: Srv.controller.ClearMap.Response) => {
			if (response.success) {
				// 删除地图成功
				nodeInfo.value = {
					state: NodeState.Wait,
					task: '',
					feedback: '删除地图成功',
					result: 'success',
					percentage: 100,
					cancel: null
				}
				resolve(response);
			} else {
				nodeInfo.value = {
					state: NodeState.Wait,
					task: '',
					feedback: '删除地图失败',
					result: 'fail',
					percentage: 100,
					cancel: null
				}
				console.warn('clear map failed')
				reject('clear map failed');
			}
		}, (error: any) => {
			nodeInfo.value = {
				state: NodeState.Wait,
				task: '',
				feedback: '删除地图失败',
				result: 'error',
				percentage: 100,
				cancel: null
			}
			console.warn(error);
			reject(error);
		})
	});
	
}

// 自动设置机器人位置
const autoInitPoseService = new ROSLIB.Service({
    ros: ros,
    name: '/ctrl/pending/auto_init_pose',
    serviceType: '/controller/AutoInitPose'
})
function autoInitPose(): Promise<any> {
    nodeInfo.value = {
        state: NodeState.Auto_Init_Pose,
        task: '自动设置机器人位置',
        feedback: '正在自动设置机器人位置...',
        result: '',
        percentage: -1,
        cancel: null
    }
	const requset: Srv.controller.AutoInitPose.Request = {
	    caller: 'frontend'
	}
	return new Promise((resolve, reject) => {
		autoInitPoseService.callService(requset, (response: Srv.controller.AutoInitPose.Response) => {
			if (response.success) {
				// 自动设置机器人位置成功
				nodeInfo.value = {
					state: NodeState.Wait,
					task: '',
					feedback: '自动设置机器人位置成功',
					result: 'success',
					percentage: 100,
					cancel: null
				}
				resolve(response);
			} else {
				nodeInfo.value = {
					state: NodeState.Wait,
					task: '',
					feedback: '自动设置机器人位置失败',
					result: 'fail',
					percentage: 100,
					cancel: null
				}
				console.warn('auto init pose failed');
				reject('auto init pose failed');
			}
		}, (error: any) => {
			nodeInfo.value = {
				state: NodeState.Wait,
				task: '',
				feedback: '自动设置机器人位置失败',
				result: 'error',
				percentage: 100,
				cancel: null
			}
			console.warn(error);
			reject(error);
		});
	});
	
}

// 手动设置机器人位置
const manualInitPoseService = new ROSLIB.Service({
    ros: ros,
    name: '/ctrl/pending/manual_init_pose',
    serviceType: '/controller/ManualInitPose'
})
function manualInitPose(pose: Msg.geometry.Pose): Promise<any> {
    nodeInfo.value = {
        state: NodeState.Manual_Init_Pose,
        task: '手动设置机器人位置',
        feedback: '正在手动设置机器人位置...',
        result: '',
        percentage: -1,
        cancel: null
    }
	const requset: Srv.controller.ManualInitPose.Request = {
	    pose: pose
	}
	return new Promise((resolve, reject) => {
		manualInitPoseService.callService(requset, (response: Srv.controller.ManualInitPose.Response) => {
			if (response.success) {
				// 手动设置机器人位置成功
				nodeInfo.value = {
					state: NodeState.Wait,
					task: '',
					feedback: '手动设置机器人位置成功',
					result: 'success',
					percentage: 100,
					cancel: null
				}
				resolve(response);
			} else {
				nodeInfo.value = {
					state: NodeState.Wait,
					task: '',
					feedback: '手动设置机器人位置失败',
					result: 'fail',
					percentage: 100,
					cancel: null
				}
				console.warn('manual init pose failed');
				reject('manual init pose failed');
			}
		}, (error: any) => {
			nodeInfo.value = {
				state: NodeState.Wait,
				task: '',
				feedback: '手动设置机器人位置失败',
				result: 'error',
				percentage: 100,
				cancel: null
			}
			console.warn(error);
			reject(error);
		});
	});
	
}

// 保存地图
const saveMapService = new ROSLIB.Service({
    ros: ros,
    name: '/ctrl/pending/save_map',
    serviceType: '/controller/SaveMap'
})
function saveMap(): Promise<any> {
    nodeInfo.value = {
        state: NodeState.Save_Map,
        task: '保存地图',
        feedback: '正在保存地图...',
        result: '',
        percentage: -1,
        cancel: null
    };
	const requset: Srv.controller.SaveMap.Request = {
	    caller: 'frontend'
	};
	return new Promise((resolve, reject) => {
		saveMapService.callService(requset, (response: Srv.controller.SaveMap.Response) => {
			if (response.success) {
				// 保存地图成功
				nodeInfo.value = {
					state: NodeState.Wait,
					task: '',
					feedback: '保存地图成功',
					result: 'success',
					percentage: 100,
					cancel: null
				}
				resolve(response);
			} else {
				nodeInfo.value = {
					state: NodeState.Wait,
					task: '',
					feedback: '保存地图失败',
					result: 'fail',
					percentage: 100,
					cancel: null
				}
				console.warn('save map failed');
				reject('save map failed');
			}
		}, (error: any) => {
			nodeInfo.value = {
				state: NodeState.Wait,
				task: '',
				feedback: '保存地图失败',
				result: 'error',
				percentage: 100,
				cancel: null
			}
			console.warn(error);
			reject(error);
		});
	});
	
}


// 导航
const navigateActionClient = new ROSLIB.ActionClient({
    ros: ros,
	serverName: '/ctrl/pending/navigate',
	actionName: 'navigation/NavigateAction',
})
const feedbackInfo: Record<string, string> = {
	'success': '导航成功',
	'fail': '导航失败',
	'error': '导航异常',
	'cancel': '导航取消',
    'normal': '正常',
	'barrier': '遇到障碍',
	'planning': '正在规划路径',
}
function navigate(pos: Msg.geometry.Pose): Promise<any> {
    const goal = new ROSLIB.Goal({
		actionClient: navigateActionClient,
		goalMessage: {
			pos: pos
		} as Action.Navigation.Navigate.Goal
	});
	nodeInfo.value = {
	    state: NodeState.Navigate,
	    task: '导航',
	    feedback: '正在导航...',
	    result: '',
	    percentage: 0,
	    cancel: goal.cancel.bind(goal)
	}
	return new Promise((resolve, reject) => {
		goal.on('result', (result: Action.Navigation.Navigate.Result) => {
			nodeInfo.value = {
				state: NodeState.Wait,
				task: '',
				feedback: feedbackInfo[result.result],
				result: result.result,
				percentage: 100,
				cancel: null
			}
			if (result.result === 'error')
				reject(result);
			else resolve(result)
		});
		goal.on('feedback', (feedback: Action.Navigation.Navigate.Feedback) => {
			nodeInfo.value = {
				state: NodeState.Navigate,
				task: '导航',
				feedback: feedbackInfo[feedback.cur_state],
				result: '',
				percentage: feedback.percentage,
				cancel: goal.cancel.bind(goal)
			}
		})
		goal.send();
	})
}

export { 
	clearMap,
	autoInitPose,
	manualInitPose,
	saveMap,
	navigate,
}