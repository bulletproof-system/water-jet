import * as ROSLIB from 'roslib';
import { Msg, Srv, ros } from '.';
import * as THREE from 'three';
import { useAppStore } from '@/stores/app';
// import { PCDLoader } from 'three/examples/jsm/loaders/PCDLoader';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader';


const appStore = useAppStore()

const sphere = new THREE.SphereGeometry();
const object = new THREE.Mesh( sphere, new THREE.MeshBasicMaterial( ) );
const box = new THREE.BoxHelper( object, 0xffff00 );

const loader = new GLTFLoader();
const flowerpots = new THREE.Group();
let potModel = new THREE.Group();
new Promise(() => {
	loader.load('watercolor_succulent.glb',
		(gltf) => {
			potModel = gltf.scene
		}, undefined,
		(error) => {
			console.error(error);
		}
	)
})


export class Pot {
	id: string // 花盆 id
	pose: Msg.geometry.Pose // 世界坐标
	// data: Uint8Array // 点云数据 
	picture: Uint8Array // 花照片
	active: boolean // 是否自动浇灌
	choose: boolean // 是否被选中
	last_water_date: Date; // 上次浇水时间
	pointCloud: THREE.Object3D // 3d 模型

	constructor(info: Msg.database.PotInfo) {
		this.id = info.id.toString();
		this.choose = false;
		this.update(info)
	}
	update(info: Msg.database.PotInfo) {
		this.pose = info.pose;
		// this.data = info.data;
		this.picture = info.picture;
		this.active = info.active;
		if (!this.active) this.choose = false;
		this.last_water_date = info.last_water_date;

		// 更新点云
		this.delete()

		try {
			const geometry = new THREE.IcosahedronGeometry( 0.3 ); 
			const material = new THREE.MeshBasicMaterial( {color: 0x00ff00} ); 
			const capsule = new THREE.Mesh( geometry, material ); 
			this.pointCloud = capsule;
			// this.pointCloud = potModel.clone();
			this.pointCloud.userData = {
				id: this.id
			}
			this.pointCloud.position.copy(this.pose.position);
			this.pointCloud.quaternion.copy(this.pose.orientation);
			this.pointCloud.visible = this.active;
			flowerpots.add(this.pointCloud);
		} catch {
			console.error(`parse point cloud failed, id: ${this.id}`);
		}

		// 更新图片
		

		// 展示点云
		
	}
	delete() {
		if (this.pointCloud) {
			this.pointCloud.clear();
			flowerpots.remove(this.pointCloud);
			this.pointCloud = null;
		}
	}
	setActive(active: boolean) {
		setPotActive(this.id, active).then(() => {
			if (this.pointCloud) {
				this.pointCloud.visible = active;
			}
		});
	}
	setChoose(choose: boolean) {
	    this.choose = choose;
	}
	locate() {

	}
}

const pots = ref(new Map<string, Pot>())
// for (let i=1; i<=10; i+=2) {
// 	pots.value.set(i.toString(), new Pot({
// 		id: i,
// 		pose: {
// 			position: { x: Math.random() * 3 - 6, y: Math.random() * 3 - 6, z: Math.random() * 3 },
// 			orientation: { x: 0, y: 0, z: 0, w: 1 }
// 		},
// 		data: undefined,
// 		picture: undefined,
// 		active: true,
// 		last_water_date: new Date(),
// 	}))
// }

const potUpdateTopic = new ROSLIB.Topic({
    ros: ros,
	name: '/database/pot/update',
	messageType: 'database/PotUpdate'
})
const potListService = new ROSLIB.Service({
	ros: ros,
	name: '/database/pot/list',
	serviceType: 'database/PotList'
});
const getPotInfoService = new ROSLIB.Service({
	ros: ros,
	name: '/database/pot/get',
	serviceType: 'database/GetPotInfo'
})
const setPotActiveService = new ROSLIB.Service({
	ros: ros,
	name: '/database/pot/set_active',
	serviceType: 'database/SetPotActive'
})


ros.on('connection', () => {
	const request: Srv.database.PotList.Request = { caller: 'frontend' };
	potListService.callService(request, (response: Srv.database.PotList.Response) => {
		pots.value.forEach((pot) => {
			pot.delete()
		})
		pots.value.clear()
		for (const pot of response.pots) {
		    pots.value.set(pot.id.toString(), new Pot(pot));
		}
	})
})

// 更新花盆数据
potUpdateTopic.subscribe(updatePots);
function updatePots(message: ROSLIB.Message) {
	const {update_list: updateList, delete_list: deleteList} = message as unknown as Msg.database.PotUpdate;
	removePot(...deleteList);
	updateList.forEach(id => fetchPotInfo(id));
}
function fetchPotInfo(id: number) {
	const request: Srv.database.GetPotInfo.Request = { id: id };
	getPotInfoService.callService(request, (response: Srv.database.GetPotInfo.Response) => {
		if (!response.success) {
			console.error(`fetch pot info failed, id: ${id}`);
			return;
		}
		const potInfo = response.info;
		updatePot(potInfo);
	});
}
function updatePot(pot: Msg.database.PotInfo) {
	if (pots.value.has(pot.id.toString()))
		pots.value[pot.id.toString()].update(pot);
	else
		this.pots.set(pot.id, pot);
};
function removePot(...ids: number[]) {
	for (const id of ids) {
		pots.value.get(id.toString()).delete()
		pots.value.delete(id.toString())
	}
}

// 设置花盆 active 
function setPotActive(id: string, active: boolean): Promise<any> {
    const request: Srv.database.SetPotActive.Request = { id: parseInt(id), active: active };
	return new Promise((resolve, reject) => {
		setPotActiveService.callService(request, (response: Srv.database.SetPotActive.Response) => {
			if (!response.success) {
				console.warn(`set pot active failed, id: ${id}`);
				reject(false);
				return;
			}
			resolve(true)
		}, (error) => {
			console.error(`set pot active error, id: ${id}`);
			reject(error)
			return;
		});
		if (appStore.debug) {
			setTimeout(() => {
				pots.value.get(id).active = active;
				resolve(true);
			}, 1000)
		}
	})
}

export { pots, flowerpots }