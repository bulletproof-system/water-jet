import { Msg } from ".."

export namespace Navigation {
	// /navigation/action/Navigate.action
	export namespace Navigate {
		export interface Goal {
			pos: Msg.geometry.Pose
		}
		export interface Result {
		    result: 'success' | 'fail' | 'cancel' | 'error'
		}
		export interface Feedback {
		    percentage: number,
			cur_state: 'normal' | 'barrier',
		}
	}
}

export namespace MapProvider {
	// /map_provider/action/InitMap.action
	export namespace InitMap {
		export interface Goal {
			caller: string
		}
		export interface Result {
			result: 'success' | 'fail' | 'cancel' | 'error'
		}
		export interface Feedback {
			percentage: number,
		}
	}
}

export namespace Controller {
	// /controller/action/Inspect.action
	export namespace Inspect {
		export interface Goal {
			caller: string
		}
		export interface Result {
		    result: 'success' | 'fail' | 'cancel' | 'error'
		}
		export interface Feedback {
		    percentage: number,
			target: number
		}
	}

	// /controller/action/Target.action
	export namespace Target {
		export interface Goal {
		    targets: string[]
		}
		export interface Result {
		    result: 'success' | 'fail' | 'cancel' | 'error'
		}
		export interface Feedback {
		    percentage: number,
			target: string
		}
	}

	// /controller/action/AutoWater.action
	export namespace AutoWater {
		export interface Goal {
		    caller: string
		}
		export interface Result {
		    result: 'success' | 'fail' | 'cancel' | 'error'
		}
		export interface Feedback {
		    percentage: number, 
			target: number // 目标花盆 id
		}
	}
}