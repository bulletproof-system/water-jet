import { ros, Action } from "..";
import { useROSStore, NodeState } from "@/stores/ros";
import * as ROSLIB from "roslib";

const rosStore = useROSStore();
const { ctrlMode, nodeInfo } = storeToRefs(rosStore)

const autoInitMapActionClient = new ROSLIB.ActionClient({
    ros: ros,
	serverName: '/ctrl/auto_map/auto_init_map',
	actionName: 'map_provider/InitMapAction',
})
const feedbackInfo: Record<string, string> = {
	'success': '建图成功',
	'fail': '建图失败',
	'error': '建图异常',
	'cancel': '建图取消',
}
function autoInitMap(): Promise<any> {
	let goal = new ROSLIB.Goal({
	    actionClient: autoInitMapActionClient,
		goalMessage: {
			caller: 'frontend'
		} as Action.MapProvider.InitMap.Goal
	});
	rosStore.setNodeInfo({
		feedback: '正在建图...',
	    result: '',
	    percentage: -1,
	    cancel: goal.cancel.bind(goal)
	})
	return new Promise((resolve, reject) => {
		goal.on('result', (result: Action.MapProvider.InitMap.Result) => {
			rosStore.setNodeInfo({
				feedback: feedbackInfo[result.result],
				result: result.result,
				percentage: 100,
				cancel: null
			})
			goal = null;
			if (result.result === 'error')
				reject(result);
			else resolve(result)
		});
		goal.on('feedback', (feedback: Action.MapProvider.InitMap.Feedback) => {
			rosStore.setNodeInfo({
				feedback: '正在建图...',
				result: '',
				percentage: feedback.percentage,
				cancel: goal.cancel.bind(goal)
			})
		});
		goal.send();
	})
}

export { autoInitMap }