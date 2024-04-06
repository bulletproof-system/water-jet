export interface TimeMsg {
	sec: number
	nsec: number
}

// https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html
export interface HeaderMsg {
	seq: number
	stamp: TimeMsg
	frame_id: string
}

// https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html
export interface PointMsg {
	x: number
	y: number
	z: number
}

// https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Vector3.html
export interface Vector3Msg {
	x: number
	y: number
	z: number
}

// https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Quaternion.html
export interface QuaternionMsg {
    x: number
	y: number
	z: number
	w: number
}

// https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html
export interface PoseMsg {
	position: PointMsg
	orientation: QuaternionMsg
}

// https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/MapMetaData.html
export interface MapMetaDateMsg {
	map_load_time: TimeMsg
	resolution: number
	width: number
	height: number
	origin: PoseMsg
}

// https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html
export interface OccupancyGridMsg {
	header: HeaderMsg
	info: MapMetaDateMsg
	data: number[]
}

// https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Transform.html
export interface TransformMsg {
    translation: Vector3Msg
	rotation: QuaternionMsg
}

// https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
export interface LaserScanMsg {
	header: HeaderMsg
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

// https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html
export interface PoseStampedMsg {
	header: HeaderMsg
	pose: PoseMsg
}

export interface PathMsg {
	header: HeaderMsg
	poses: PoseStampedMsg[]
}