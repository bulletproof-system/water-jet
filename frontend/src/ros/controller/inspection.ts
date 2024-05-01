import { useAppStore } from "@/stores/app";
import { ros, Action } from "..";
import { useROSStore, NodeState } from "@/stores/ros";
import * as ROSLIB from "roslib";

const appStore = useAppStore();
const rosStore = useROSStore();
const { ctrlMode, nodeInfo } = storeToRefs(rosStore)

const inspectActionClient = new ROSLIB.ActionClient({
    ros: ros,
	serverName: '/ctrl/inspection/inspect',
	actionName: 'controller/Inspect'
})
const feedbackInfo: Record<string, string> = {
	'success': '巡检完成',
	'fail': '巡检失败',
	'error': '巡检异常',
	'cancel': '巡检取消',
}
function inspect(): () => void {
	const goal = new ROSLIB.Goal({
	    actionClient: inspectActionClient,
		goalMessage: {
			caller: 'frontend'
		} as Action.Controller.Inspect.Goal
	});
	nodeInfo.value = {
		state: NodeState.Inspect,
		task: '缺水巡检',
		feedback: '巡检中...',
		result: '',
		percentage: 0,
		cancel: goal.cancel
	};
	goal.on('result', (result: Action.Controller.Inspect.Result) => {
		nodeInfo.value = {
			state: NodeState.Wait,
			task: '',
			feedback: feedbackInfo[result.result],
			result: result.result,
			percentage: 100,
			cancel: null
		}
	});
	goal.on('feedback', (feedback: Action.Controller.Inspect.Feedback) => {
	    nodeInfo.value = {
			state: NodeState.Inspect,
			task: '缺水巡检',
			feedback: `当前目标花盆 id: ${feedback.target}`,
			result: '',
			percentage: feedback.percentage,
			cancel: goal.cancel
		}
		appStore.openPot(feedback.target.toString());
		appStore.locatePot(feedback.target.toString());
	});
	goal.send();
	return goal.cancel;
}

export { inspect }