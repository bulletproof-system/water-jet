import { CtrlMode, NodeState } from "@/stores/ros"

export interface Time {
	sec: number
	nsec: number
}

// https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html
export interface Header {
	seq: number
	stamp: Time
	frame_id: string
}

export namespace geometry {
	// https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html
	export interface Point {
		x: number
		y: number
		z: number
	}

	// https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Vector3.html
	export interface Vector3 {
		x: number
		y: number
		z: number
	}

	// https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Quaternion.html
	export interface Quaternion {
		x: number
		y: number
		z: number
		w: number
	}

	// https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html
	export interface Pose {
		position: Point
		orientation: Quaternion
	}

	// https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Transform.html
	export interface Transform {
		translation: Vector3
		rotation: Quaternion
	}

	// https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html
	export interface PoseStamped {
		header: Header
		pose: Pose
	}

	// https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html
	export interface PointStamped {
		header: Header
		point: Point
	}
}

export namespace nav {
	// https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/MapMetaData.html
	export interface MapMetaDate {
		map_load_time: Time
		resolution: number
		width: number
		height: number
		origin: geometry.Pose
	}

	// https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html
	export interface OccupancyGrid {
		header: Header
		info: MapMetaDate
		data: number[]
	}
	
	// https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html
	export interface Path {
		header: Header
		poses: geometry.PoseStamped[]
	}
}

export namespace sensor {
	// https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
	export interface LaserScan {
		header: Header
		angle_min: number
		angle_max: number
		angle_increment: number
		time_increment: number
		scan_time: number
		range_min: number
		range_max: number
		ranges: number[]
		intensities: number[]
	}
	// https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointField.html
	export interface PointField {
	    name: string 
		offset: number
		datatype: number
		count: number
	}

	// https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html
	export interface PointCloud2 {
	    header: Header
		height: number
		width: number
		fields: PointField[]
		is_bigendian: boolean
		point_step: number
		row_step: number
		data: Uint8Array
		is_dense: boolean
	}

	// https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
	export interface Image {
		header: Header
		height: number
		width: number
		encoding: string
		is_bigendian: boolean
		step: number
		data: Uint8Array
	}
}

export namespace database {
	// database/msg/PotInfo.msg
	export interface PotInfo {
		id: number // 花盆 id
		pot_pose: geometry.Pose // 世界坐标
		robot_pose: geometry.Pose // 世界坐标
		data: Uint8Array // 点云数据 
		picture: string // 花照片
		active: boolean // 是否自动浇灌
		last_water_date: string // 上次浇水时间
	}

	// /database/msg/PotUpdate.msg
	export interface PotUpdate {
	    update_list: number[] // 更新的花盆 id 列表
		delete_list: number[] // 删除的花盆 id 列表
	}
}


export namespace controller {
	// /controller/msg/Hello.msg
	export interface Hello {
	    publisher: string
	}

	// /controller/msg/Info.msg
	export interface Info {
		header: Header,
		mode: CtrlMode,
		scram: boolean,
	}

	// /controller/msg/NodeInfo.msg 
	export interface NodeInfo {
	    state: NodeState,
		mode: CtrlMode,
	}
}