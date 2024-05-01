import { ros, Action } from "..";
import { useROSStore, NodeState } from "@/stores/ros";
import * as ROSLIB from "roslib";

const rosStore = useROSStore();
const { ctrlMode, nodeInfo } = storeToRefs(rosStore)

const manualInitMapActionClient = new ROSLIB.ActionClient({
    ros: ros,
	serverName: '/ctrl/manual_map/manual_init_map',
	actionName: 'map_provider/InitMap',
})
const feedbackInfo: Record<string, string> = {
	'success': '导航成功',
	'fail': '导航失败',
	'error': '导航异常',
	'cancel': '导航取消',
}
function manualInitMap(): ()=>void {
	const goal = new ROSLIB.Goal({
	    actionClient: manualInitMapActionClient,
		goalMessage: {
			caller: 'frontend'
		} as Action.MapProvider.InitMap.Goal
	});
	nodeInfo.value = {
		state: NodeState.Manual_Init_Map,
		task: '手动建图',
	    feedback: '正在建图...',
	    result: '',
	    percentage: -1,
	    cancel: goal.cancel
	}
	goal.on('result', (result: Action.MapProvider.InitMap.Result) => {
		nodeInfo.value = {
			state: NodeState.Wait,
			task: '',
		    feedback: feedbackInfo[result.result],
		    result: result.result,
		    percentage: -1,
		    cancel: null
		} 
	});
	goal.on('feedback', (feedback: Action.MapProvider.InitMap.Feedback) => {
		nodeInfo.value = {
			state: NodeState.Manual_Init_Map,
			task: '手动建图',
			feedback: '正在建图...',
			result: '',
			percentage: feedback.percentage,
			cancel: goal.cancel
		};
	});
	goal.send();
	return goal.cancel;
}

export { manualInitMap }