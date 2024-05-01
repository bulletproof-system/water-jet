import { ros, TF, Srv, Msg } from '@/ros'
import * as ROSLIB from 'roslib';
import * as THREE from 'three';

let mapService = new ROSLIB.Service({
	ros: ros,
	name: '/control/create_map/start',
	serviceType: 'create_map/Base'
})
let mapListener = new ROSLIB.Topic<Msg.nav.OccupancyGrid>({
	ros: ros,
	name: '/map',
	messageType: 'nav_msgs/OccupancyGrid',
	throttle_rate: 1000,
	compression: 'cbor'
});
let mapInfo = {
	width: 10,
	height: 1,
	resolution: 1,
	origin: {
		position: new THREE.Vector3(),
		orientation: new THREE.Quaternion()
	}
}
let globalMap = new THREE.Group()
let occupancyGrid: Msg.nav.OccupancyGrid = null
let color = new Uint8Array(1 * 1 * 4)
let mapTexture: THREE.DataTexture = null
let mapGeometry = new THREE.PlaneGeometry(10, 1)
let mapMaterial = new THREE.MeshBasicMaterial({ side: THREE.FrontSide, map: mapTexture })
let mapGrid = new THREE.Mesh(mapGeometry, mapMaterial)
globalMap.add(mapGrid)

startCreateMap()
mapListener.subscribe(procMapMsg);

function startCreateMap() {
	const req: Srv.Base.Request = {
		request: 'start'
	}
	mapService.callService(req, (res: Srv.Base.Response) => {
		console.log(res)
	})
}

function procMapMsg(message: unknown) {
	function syncMapMetaData() {
		const info = occupancyGrid.info
		if (info.width != mapInfo.width ||
			info.height != mapInfo.height ||
			info.resolution != mapInfo.resolution ||
			info.origin.position.x != mapInfo.origin.position.x ||
			info.origin.position.y != mapInfo.origin.position.y ||
			info.origin.position.z != mapInfo.origin.position.z ||
			info.origin.orientation.x != mapInfo.origin.orientation.x ||
			info.origin.orientation.y != mapInfo.origin.orientation.y ||
			info.origin.orientation.z != mapInfo.origin.orientation.z ||
			info.origin.orientation.w != mapInfo.origin.orientation.w) {
			mapInfo.width = info.width
			mapInfo.height = info.height
			mapInfo.resolution = info.resolution
			mapInfo.origin.position = new THREE.Vector3(info.origin.position.x, info.origin.position.y, info.origin.position.z)
			mapInfo.origin.orientation = new THREE.Quaternion(info.origin.orientation.x, info.origin.orientation.y, info.origin.orientation.z, info.origin.orientation.w)
			return true
		}
		else return false
	}
	occupancyGrid = message as unknown as Msg.nav.OccupancyGrid;
	if (syncMapMetaData()) {
		globalMap.remove(mapGrid)
		color = new Uint8Array(mapInfo.width * mapInfo.height * 4);
		mapGeometry = new THREE.PlaneGeometry(mapInfo.width, mapInfo.height);
		mapTexture = new THREE.DataTexture(color, mapInfo.width, mapInfo.height, THREE.RGBAFormat);
		mapMaterial = new THREE.MeshBasicMaterial({ side: THREE.FrontSide, map: mapTexture })
		mapGrid = new THREE.Mesh(mapGeometry, mapMaterial)
		const offset = new THREE.Vector3(mapInfo.width / 2, mapInfo.height / 2, 0).multiplyScalar(mapInfo.resolution).add(mapInfo.origin.position);
		mapGrid.applyMatrix4(new THREE.Matrix4().compose(offset, mapInfo.origin.orientation, new THREE.Vector3(mapInfo.resolution, mapInfo.resolution, 1)))
		globalMap.add(mapGrid)
	}
	let id = 0
	for (let i = 0; i < mapInfo.width; ++i) {
		for (let j = 0; j < mapInfo.height; ++j, ++id) {
			const c = getColor(occupancyGrid.data[id])
			color[id * 4] = c[0];
			color[id * 4 + 1] = c[1];
			color[id * 4 + 2] = c[2];
			color[id * 4 + 3] = c[3];
		}
	}
	mapTexture.needsUpdate = true;
}

function hslToRgba(h: number, s: number, l: number) {
	h /= 360;
	s /= 100;
	l /= 100;

	let r: number, g: number, b: number;

	if (s === 0) {
		r = g = b = l; // achromatic
	} else {
		const hue2rgb = (p: number, q: number, t: number) => {
			if (t < 0) t += 1;
			if (t > 1) t -= 1;
			if (t < 1 / 6) return p + (q - p) * 6 * t;
			if (t < 1 / 2) return q;
			if (t < 2 / 3) return p + (q - p) * (2 / 3 - t) * 6;
			return p;
		};

		const q = l < 0.5 ? l * (1 + s) : l + s - l * s;
		const p = 2 * l - q;
		r = hue2rgb(p, q, h + 1 / 3);
		g = hue2rgb(p, q, h);
		b = hue2rgb(p, q, h - 1 / 3);
	}

	return [r * 256, g * 256, b * 256, 255].map((c) => Math.round(c));
}

function getColor(lightness: number) {
	if (lightness < 0) return [255, 255, 255, 0]
	return hslToRgba(180, 100, 40 - 0.4 * lightness);
}


export { globalMap }