import { useAppStore } from "@/stores/app";
import { ros, Action, pots } from "..";
import { useROSStore, NodeState } from "@/stores/ros";
import { useMapStore } from "@/stores/map";
import * as ROSLIB from "roslib";

const appStore = useAppStore();
const rosStore = useROSStore();
const mapStore = useMapStore();
const { ctrlMode, nodeInfo } = storeToRefs(rosStore)

const targetActionClient = new ROSLIB.ActionClient({
    ros: ros,
	serverName: '/ctrl/target/target',
	actionName: 'controller/TargetAction'
})
const feedbackInfo: Record<string, string> = {
	'success': '浇水完成',
	'fail': '任务失败',
	'error': '任务异常',
	'cancel': '任务取消',
}
function target(): Promise<any> {
	const targets = (() => {
		let res = [];
		pots.value.forEach((pot, id) => {
		    if (pot.choose) res.push(id);
		})
		return res;
	})()
	let goal = new ROSLIB.Goal({
	    actionClient: targetActionClient,
		goalMessage: {
			targets: targets,
		} as Action.Controller.Target.Goal
	});
	rosStore.setNodeInfo({
		feedback: '初始化...',
		result: '',
		percentage: 0,
		cancel: goal.cancel.bind(goal)
	})
	return new Promise((resolve, reject) => {
		if (targets.length === 0) {
			rosStore.setNodeInfo({
				feedback: '请选择目标花盆',
				result: '',
				percentage: 0,
				cancel: null
			})
			reject('no target');
			return;
		}
		goal.on('result', (result: Action.Controller.Target.Result) => {
			rosStore.setNodeInfo({
				feedback: feedbackInfo[result.result],
				result: result.result,
				percentage: 100,
				cancel: null
			})
			goal = null;
			if (result.result === 'error')
				reject(result);
			else resolve(result);
		});
		goal.on('feedback', (feedback: Action.Controller.Target.Feedback) => {
			rosStore.setNodeInfo({
				feedback: `当前目标花盆 id: ${feedback.target}`,
				result: '',
				percentage: feedback.percentage,
				cancel: goal.cancel.bind(goal)
			})
			appStore.openPot(feedback.target);
			mapStore.locatePot(feedback.target);
		});
		goal.send();
	})
}

export { target }