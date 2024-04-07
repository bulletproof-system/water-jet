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

// https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/MapMetaData.html
export interface MapMetaDate {
	map_load_time: Time
	resolution: number
	width: number
	height: number
	origin: Pose
}

// https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html
export interface OccupancyGrid {
	header: Header
	info: MapMetaDate
	data: number[]
}

// https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Transform.html
export interface Transform {
    translation: Vector3
	rotation: Quaternion
}

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

// https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html
export interface PoseStamped {
	header: Header
	pose: Pose
}

// https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html
export interface Path {
	header: Header
	poses: PoseStamped[]
}

// https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html
export interface PointStamped {
	header: Header
	point: Point
}