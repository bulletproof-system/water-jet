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
	rosStore.setNodeInfo({
	    feedback: '正在删除地图...',
		result: '',
	    percentage: -1,
		cancel: null
	})
	const requset: Srv.controller.ClearMap.Request = {
	    clear: true
	}
	return new Promise((resolve, reject) => {
		clearMapService.callService(requset, (response: Srv.controller.ClearMap.Response) => {
			if (response.success) {
				// 删除地图成功
				rosStore.setNodeInfo({
					feedback: '删除地图成功',
					result: 'success',
					percentage: 100,
					cancel: null
				})
				resolve(response);
			} else {
				rosStore.setNodeInfo({
				    feedback: '删除地图失败',
					result: 'fail',
					percentage: 100,
					cancel: null
				})
				console.warn('clear map failed')
				reject('clear map failed');
			}
		}, (error: any) => {
			rosStore.setNodeInfo({
			    feedback: '删除地图失败',
				result: 'error',
				percentage: 100,
				cancel: null
			})
			console.warn(error);
			reject(error);
		})
	});
	
}

// 自动设置机器人位置
const autoInitPosService = new ROSLIB.Service({
    ros: ros,
    name: '/ctrl/pending/auto_init_pos',
    serviceType: '/controller/AutoInitPos'
})
function autoInitPos(): Promise<any> {
	rosStore.setNodeInfo({
		feedback: '正在自动设置机器人位置...',
		result: '',
	    percentage: -1,
		cancel: null
	})
	const requset: Srv.controller.AutoInitPos.Request = {
	    caller: 'frontend'
	}
	return new Promise((resolve, reject) => {
		autoInitPosService.callService(requset, (response: Srv.controller.AutoInitPos.Response) => {
			if (response.success) {
				// 自动设置机器人位置成功
				rosStore.setNodeInfo({
					feedback: '自动设置机器人位置成功',
					result: 'success',
					percentage: 100,
					cancel: null
				})
				resolve(response);
			} else {
				rosStore.setNodeInfo({
				    feedback: '自动设置机器人位置失败',
					result: 'fail',
					percentage: 100,
					cancel: null
				})
				console.warn('auto init pose failed');
				reject('auto init pose failed');
			}
		}, (error: any) => {
			rosStore.setNodeInfo({
			    feedback: '自动设置机器人位置失败',
				result: 'error',
				percentage: 100,
				cancel: null
			})
			console.warn(error);
			reject(error);
		});
	});
	
}

// 手动设置机器人位置
const manualInitPosService = new ROSLIB.Service({
    ros: ros,
    name: '/ctrl/pending/manual_init_pos',
    serviceType: '/controller/ManualInitPos'
})
function manualInitPos(pose: Msg.geometry.Pose): Promise<any> {
	rosStore.setNodeInfo({
	    feedback: '正在手动设置机器人位置...',
		result: '',
	    percentage: -1,
		cancel: null
	})
	const requset: Srv.controller.ManualInitPos.Request = {
	    pos: pose
	}
	return new Promise((resolve, reject) => {
		manualInitPosService.callService(requset, (response: Srv.controller.ManualInitPos.Response) => {
			if (response.success) {
				// 手动设置机器人位置成功
				rosStore.setNodeInfo({
				    feedback: '手动设置机器人位置成功',
					result: 'success',
					percentage: 100,
					cancel: null
				})
				resolve(response);
			} else {
				rosStore.setNodeInfo({
				    feedback: '手动设置机器人位置失败',
					result: 'fail',
					percentage: 100,
					cancel: null
				})
				console.warn('manual init pose failed');
				reject('manual init pose failed');
			}
		}, (error: any) => {
			rosStore.setNodeInfo({
			    feedback: '手动设置机器人位置失败',
				result: 'error',
				percentage: 100,
				cancel: null
			})
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
	rosStore.setNodeInfo({
		feedback: '正在保存地图...',
		result: '',
	    percentage: -1,
	    cancel: null
	})
	const requset: Srv.controller.SaveMap.Request = {
	    caller: 'frontend'
	};
	return new Promise((resolve, reject) => {
		saveMapService.callService(requset, (response: Srv.controller.SaveMap.Response) => {
			if (response.success) {
				// 保存地图成功
				rosStore.setNodeInfo({
				    feedback: '保存地图成功',
					result: 'success',
					percentage: 100,
					cancel: null
				})
				resolve(response);
			} else {
				rosStore.setNodeInfo({
				    feedback: '保存地图失败',
					result: 'fail',
					percentage: 100,
					cancel: null
				})
				console.warn('save map failed');
				reject('save map failed');
			}
		}, (error: any) => {
			rosStore.setNodeInfo({
				feedback: '保存地图失败',
				result: 'error',
				percentage: 100,
				cancel: null
			})
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
    'normal': '正在导航',
	'barrier': '遇到障碍',
	'planning': '正在规划路径',
}
function navigate(pos: Msg.geometry.Pose): Promise<any> {
    let goal = new ROSLIB.Goal({
		actionClient: navigateActionClient,
		goalMessage: {
			pos: pos
		} as Action.Navigation.Navigate.Goal
	});
	rosStore.setNodeInfo({
	    percentage: 0,
	    cancel: goal.cancel.bind(goal)
	})
	return new Promise((resolve, reject) => {
		goal.on('result', (result: Action.Navigation.Navigate.Result) => {
			rosStore.setNodeInfo({
				feedback: feedbackInfo[result.result],
				result: result.result,
				percentage: 100,
				cancel: null
			})
			goal = null
			if (result.result === 'error')
				reject(result);
			else resolve(result)
		});
		goal.on('feedback', (feedback: Action.Navigation.Navigate.Feedback) => {
			rosStore.setNodeInfo({
			    feedback: feedbackInfo[feedback.cur_state],
				result: '',
				percentage: feedback.percentage,
				cancel: goal.cancel.bind(goal)
			})
		})
		goal.send();
	})
}

export { 
	clearMap,
	autoInitPos as autoInitPose,
	manualInitPos as manualInitPose,
	saveMap,
	navigate,
}