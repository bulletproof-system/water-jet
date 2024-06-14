import { useAppStore } from "@/stores/app";
import { ros, Action } from "..";
import { useROSStore, NodeState } from "@/stores/ros";
import { useMapStore } from "@/stores/map";
import * as ROSLIB from "roslib";

const appStore = useAppStore();
const rosStore = useROSStore();
const mapStore = useMapStore();
const { ctrlMode, nodeInfo } = storeToRefs(rosStore)

const autoWaterActionClient = new ROSLIB.ActionClient({
    ros: ros,
    serverName: "/ctrl/auto_water/auto_water",
    actionName: "controller/AutoWaterAction"
})
const feedbackInfo: Record<string, string> = {
	'success': '巡检完成',
	'fail': '巡检失败',
	'error': '巡检异常',
	'cancel': '巡检取消',
}
function autoWater(): Promise<any> {
	let goal = new ROSLIB.Goal({
		actionClient: autoWaterActionClient,
		goalMessage: {
		    none: 'frontend'
		} as Action.Controller.AutoWater.Goal
	})
	rosStore.setNodeInfo({
		feedback: '初始化...',
		result: '',
		percentage: 0,
		cancel: goal.cancel.bind(goal)
	})
	return new Promise((resolve, reject) => {
		goal.on('result', (result: Action.Controller.AutoWater.Result) => {
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
		goal.on('feedback', (feedback: Action.Controller.AutoWater.Feedback) => {
			rosStore.setNodeInfo({
				feedback: `当前目标花盆 id: ${feedback.target}`,
				result: '',
				percentage: feedback.percentage,
				cancel: goal.cancel.bind(goal)
			})
			appStore.openPot(feedback.target.toString());
			mapStore.locatePot(feedback.target.toString());
		});
		goal.send();
	})
}

export { autoWater }