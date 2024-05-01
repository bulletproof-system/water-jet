import { defineStore } from 'pinia'

// 鼠标操作(左键)
export enum MouseAction {
	Control = 0, 		// 控制移动
	Navigate,			// 导航, 选择航点
	SetPosition,		// 设置机器人位置
}

export const useMapStore = defineStore('map', {
    state: () => ({
		mouseAction: MouseAction.Control,
    }),

	actions: {
	    setMouseAction(action: MouseAction) {
	        this.mouseAction = action
	    }
	}
})