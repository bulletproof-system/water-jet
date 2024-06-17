import { defineStore } from 'pinia'
import * as THREE from 'three'
import { string } from 'three/examples/jsm/nodes/shadernode/ShaderNode'

// 鼠标操作(左键)
export enum MouseAction {
	Control = 0, 		// 控制移动
	Navigate,			// 导航, 选择航点
	SetPosition,		// 设置机器人位置
}

// 控制模式
export enum ControlMode {
    Normal = 0, 	// 正常模式
	FollowRobot, 	// 跟随机器人
	LocatePot,		// 定位花盆
}

export interface Target {
	from?: THREE.Vector3,
	to?: THREE.Vector3,
}

export const useMapStore = defineStore('map', {
    state: () => ({
		mouseAction: MouseAction.Control,
		arrow: {
			origin: null as THREE.Vector3,
			direction: null as THREE.Vector3,
		},
		mode: ControlMode.Normal as ControlMode,
		control: {
			target: {
				from: new THREE.Vector3(),
				to: new THREE.Vector3(),
			} as Target,
			locatePot: (id: string) => console.log('unmounted'),
		},
		enableObjectDetect: false,
		yoloImage: null
    }),

	actions: {
	    setMouseAction(action: MouseAction) {
	        this.mouseAction = action
	    },
		setLocatePot(locatePot: (id: string) => void) {
			this.control.locatePot = locatePot
		},
		locatePot(id: string) {
		    this.control.locatePot(id)
		},
		setTarget(from: THREE.Vector3, to: THREE.Vector3) {
			this.control.target.from = from
			this.control.target.to = to
			this.mode = ControlMode.LocatePot
		}
	}
})