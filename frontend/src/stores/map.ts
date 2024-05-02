import { defineStore } from 'pinia'
import * as THREE from 'three'

// 鼠标操作(左键)
export enum MouseAction {
	Control = 0, 		// 控制移动
	Navigate,			// 导航, 选择航点
	SetPosition,		// 设置机器人位置
}

export const useMapStore = defineStore('map', {
    state: () => ({
		mouseAction: MouseAction.Control,
		arrow: {
			origin: null as null | THREE.Vector3,
			direction: null as null | THREE.Vector3,
		}
    }),

	actions: {
	    setMouseAction(action: MouseAction) {
	        this.mouseAction = action
	    }
	}
})