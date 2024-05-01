import { useAppStore } from "@/stores/app";
import { ros, Action, pots } from "..";
import { useROSStore, NodeState } from "@/stores/ros";
import * as ROSLIB from "roslib";

const appStore = useAppStore();
const rosStore = useROSStore();
const { ctrlMode, nodeInfo } = storeToRefs(rosStore)

const targetActionClient = new ROSLIB.ActionClient({
    ros: ros,
	serverName: '/ctrl/target/target',
	actionName: 'controller/Target'
})
const feedbackInfo: Record<string, string> = {
	'success': '浇水完成',
	'fail': '任务失败',
	'error': '任务异常',
	'cancel': '任务取消',
}
function target(): () => void {
	const targets = (() => {
		let res = [];
		pots.value.forEach((pot, id) => {
		    if (pot.choose) res.push(id);
		})
		return res;
	})()
	const goal = new ROSLIB.Goal({
	    actionClient: targetActionClient,
		goalMessage: {
			targets: targets.map((target) => parseInt(target)),
		} as Action.Controller.Target.Goal
	});
	nodeInfo.value = {
		state: NodeState.Target,
		task: '指定目标浇水',
		feedback: '执行中...',
		result: '',
		percentage: 0,
		cancel: goal.cancel
	};
	goal.on('result', (result: Action.Controller.Target.Result) => {
		nodeInfo.value = {
			state: NodeState.Wait,
			task: '',
			feedback: feedbackInfo[result.result],
			result: result.result,
			percentage: 100,
			cancel: null
		}
	});
	goal.on('feedback', (feedback: Action.Controller.Target.Feedback) => {
	    nodeInfo.value = {
			state: NodeState.Target,
			task: '指定目标浇水',
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

export { target }