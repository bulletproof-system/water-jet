import { Msg } from ".."

export namespace Base {
    export interface Request {
        request: string
    }
	export interface Response {
		response: string
	}
}

export namespace controller {

	// /controller/srv/ChangeMode.srv
	export namespace ChangeMode {
		export interface Request {
			mode: number
		}
		export interface Response {
			success: boolean
		}
	}

	// /controller/srv/Scram.srv
	export namespace Scram {
		export interface Request {
		    active: boolean
		}
		export interface Response {
		    success: boolean
		}
	}

	// /controller/srv/Start.srv 
	export namespace Start {
	    export interface Request {
	        mode: number
	    }
		export interface Response {
		    success: boolean
		}
	}

	// /controller/srv/Stop.srv
	export namespace Stop {
	    export interface Request {
	        mode: number
		}
		export interface Response {
		    success: boolean
		}
	}

	// /controller/srv/ClearMap.srv
	export namespace ClearMap {
	    export interface Request {
	        clear: boolean
		}
		export interface Response {
		    success: boolean
		}
	}

	// /controller/srv/AutoInitPose.srv
	export namespace AutoInitPose {
	    export interface Request {
	        caller: string
		}
		export interface Response {
		    success: boolean
		}
	}

	// /controller/srv/ManualInitPose.srv
	export namespace ManualInitPose {
	    export interface Request {
	        pose: Msg.geometry.Pose
		}
		export interface Response {
		    success: boolean
		}
	}

	// /controller/srv/SaveMap.srv
	export namespace SaveMap {
	    export interface Request {
	        caller: string
		}
		export interface Response {
		    success: boolean
		}
	}
}

export namespace database {
	// /database/srv/PotList.srv
    export namespace PotList {
		export interface Request {
		    caller: string
		}
		export interface Response {
			pots: Msg.database.PotInfo[]
		}
	}

	// /database/srv/SetPotInfo.srv
	export namespace SetPotInfo {

		export interface Request {
			info: Msg.database.PotInfo
		}
		export interface Response {
			success: boolean
		}
	}

	// /database/srv/SetPotActive.srv
	export namespace SetPotActive {
	    export interface Request {
	        id: number
	        active: boolean
	    }
		export interface Response {
		    success: boolean
		}
	}

	// /database/srv/GetPotInfo.srv
	export namespace GetPotInfo {
		export interface Request {
			id: number
		}
		export interface Response {
			success: boolean
			info: Msg.database.PotInfo
		}
	}

	// /database/srv/DeletePot.srv
	export namespace DeletePot {
		export interface Request {
			id: number
		}
		export interface Response {
			success: boolean
		}
	}

}