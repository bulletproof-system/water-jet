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

const flowerpots = new THREE.Group();
// loader.load('banana_plant_with_pot.glb',
// 	(gltf) => {
// 		potModel = gltf.scene
// 	}, undefined,
// 	(error) => {
// 		console.error(error);
// 	}
// )
class ModelLoaderSingleton {
	static instance: ModelLoaderSingleton;
	model: THREE.Group | null;
	loader: GLTFLoader;
	loadingLock: boolean; // 加载锁
    constructor() {
        if (!ModelLoaderSingleton.instance) {
            this.model = null;
            this.loader = new GLTFLoader();
            this.loadingLock = false; // 加载锁
            ModelLoaderSingleton.instance = this;
        }
        return ModelLoaderSingleton.instance;
    }

    async loadModel(url: string) {
		if (!this.model) {
            await new Promise((resolve, reject) => {
                this.loader.load(url, gltf => {
                    this.model = gltf.scene;
                    resolve(this.model);
                }, undefined, reject);
            });
        }
        return this.model;
    }
}

const loader = new ModelLoaderSingleton();


export class Pot {
	id: string // 花盆 id
	pot_pose: Msg.geometry.Pose // 世界坐标
	robot_pose: Msg.geometry.Pose // 世界坐标
	// data: Uint8Array // 点云数据 
	picture: string // 花照片
	active: boolean // 是否自动浇灌
	choose: boolean // 是否被选中
	last_water_date: Date; // 上次浇水时间
	pointCloud: THREE.Object3D // 3d 模型

	constructor(info: Msg.database.PotInfo) {
		this.id = info.id.toString();
		this.choose = false;
		this.update(info)
	}
	async update(info: Msg.database.PotInfo) {
		this.pot_pose = info.pot_pose;
		this.robot_pose = info.robot_pose;
		// this.data = info.data;
		this.delete()
		this.picture = "";
		try {
			let picture = new Uint8Array(info.picture.length); 
			atob(info.picture).split('').forEach((c, i) => picture[i] = c.charCodeAt(0))
			const blob = new Blob([picture], { type: 'image/jpeg' });
			this.picture = URL.createObjectURL(blob);
		} catch(error) {
			console.error(error, `parse picture failed, id: ${this.id}`);
		}
		this.active = info.active;
		if (!this.active) this.choose = false;
		if (info.last_water_date == "")
			this.last_water_date = null
		else this.last_water_date = new Date(info.last_water_date)

		// 更新点云
		try {
			this.pointCloud = (await loader.loadModel('banana_plant_with_pot.glb')).clone();
			// this.pointCloud = potModel.clone();
			this.pointCloud.userData = {
				id: this.id
			}
			this.pointCloud.position.copy(this.pot_pose.position);
			this.pointCloud.quaternion.copy(this.pot_pose.orientation);
			this.pointCloud.scale.set(0.4, 0.4, 0.4);
			this.pointCloud.applyQuaternion(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2));
			this.pointCloud.visible = this.active;
			flowerpots.add(this.pointCloud);
		} catch {
			console.error(`parse point cloud failed, id: ${this.id}`);
		}

		// 更新图片
		

		// 展示点云
		
	}
	delete() {
		URL.revokeObjectURL(this.picture)
		if (this.pointCloud) {
			console.log("delete point cloud")
			this.pointCloud.visible = false;
			this.pointCloud.removeFromParent();
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
if (appStore.debug) {
	for (let i=1; i<=10; i+=2) {
		pots.value.set(i.toString(), new Pot({
			id: i,
			pot_pose: {
				position: { x: Math.random() * 3 - 6, y: Math.random() * 3 - 6, z: Math.random() * 3 },
				orientation: { x: 0, y: 0, z: 0, w: 1 }
			},
			robot_pose: {
				position: { x: Math.random() * 3 - 6, y: Math.random() * 3 - 6, z: Math.random() * 3 },
				orientation: { x: 0, y: 0, z: 0, w: 1 }
			},
			data: undefined,
			picture: undefined,
			active: true,
			last_water_date: "2024-01-01 00:00:00",
		}))
	}
}


const potUpdateTopic = new ROSLIB.Topic({
    ros: ros,
	name: '/database/pot/update',
	messageType: 'pot_database/PotUpdate'
})
const potListService = new ROSLIB.Service({
	ros: ros,
	name: '/database/pot/list',
	serviceType: 'pot_database/PotList'
});
const getPotInfoService = new ROSLIB.Service({
	ros: ros,
	name: '/database/pot/get',
	serviceType: 'pot_database/GetPotInfo'
})
const setPotActiveService = new ROSLIB.Service({
	ros: ros,
	name: '/database/pot/set_active',
	serviceType: 'pot_database/SetPotActive'
})


ros.on('connection', () => {
	const request: Srv.database.PotList.Request = { };
	potListService.callService(request, (response: Srv.database.PotList.Response) => {
		let removeList = [];
		pots.value.forEach((_, key) => {
		    removeList.push(key);
		})
		removePot(...removeList)
		for (const pot of response.pots) {
		    pots.value.set(pot.id.toString(), new Pot(pot));
		}
	})
})

// 更新花盆数据
potUpdateTopic.subscribe(updatePots);
function updatePots(message: ROSLIB.Message) {
	const {update_list: updateList, delete_list: deleteList} = message as unknown as Msg.database.PotUpdate;
	removePot(...deleteList.map(id => id.toString()));
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
		pots.value.set(pot.id.toString(), new Pot(pot));
};
function removePot(...ids: string[]) {
	for (const id of ids) {
		pots.value.get(id).delete()
		pots.value.delete(id)
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