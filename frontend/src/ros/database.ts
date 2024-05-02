import * as ROSLIB from 'roslib';
import { Msg, Srv, ros } from '.';
import * as THREE from 'three';
import { useAppStore } from '@/stores/app';

const appStore = useAppStore()
export class Pot {
	id: string // 花盆 id
	pose: Msg.geometry.Pose // 世界坐标
	data: Msg.sensor.PointCloud2 // 点云数据 
	picture: Msg.sensor.Image // 花照片
	active: boolean // 是否自动浇灌
	choose: boolean // 是否被选中
	last_water_date: Date; // 上次浇水时间
	pointCloud: THREE.BufferGeometry // 3d 模型

	constructor(info: Msg.database.PotInfo) {
		this.id = info.id.toString();
		this.pointCloud = new THREE.BufferGeometry();
		this.choose = false;
		this.update(info)
	}
	update(info: Msg.database.PotInfo) {
		this.pose = info.pose;
		this.data = info.data;
		this.picture = info.picture;
		this.active = info.active;
		if (!this.active) this.choose = false;
		this.last_water_date = info.last_water_date;

		// 更新点云
		this.pointCloud.dispose();
		this.pointCloud = new THREE.BufferGeometry();

	}
	delete() {
		this.pointCloud.dispose();
	}
	setActive(active: boolean) {
		setPotActive(this.id, active);
	}
	setChoose(choose: boolean) {
	    this.choose = choose;
	}
}

const pots = ref(new Map<string, Pot>())
for (let i=1; i<=40; i+=2) {
	pots.value.set(i.toString(), new Pot({
		id: i,
		pose: {
			position: { x: 0, y: 0, z: 0 },
			orientation: { x: 0, y: 0, z: 0, w: 1 }
		},
		data: undefined,
		picture: undefined,
		active: true,
		last_water_date: new Date(),
	}))
}

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
const flowerpots = new THREE.Group();

ros.on('connection', () => {
	const request: Srv.database.PotList.Request = { caller: 'frontend' };
	potListService.callService(request, (response: Srv.database.PotList.Response) => {
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
function setPotActive(id: string, active: boolean) {
    const request: Srv.database.SetPotActive.Request = { id: parseInt(id), active: active };
    setPotActiveService.callService(request, (response: Srv.database.SetPotActive.Response) => {
        if (!response.success) {
            console.error(`set pot active failed, id: ${id}`);
            return;
        }
    });
	if (appStore.debug) {
		setTimeout(() => {
			pots.value.get(id).active = active
		}, 1000)
	}
}



export { pots, flowerpots }