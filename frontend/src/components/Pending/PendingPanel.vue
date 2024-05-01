<template>
	<div class="d-flex flex-column ">
		<v-btn class="ma-2 bg-primary" size="large" :disabled="nodeInfo.state !== NodeState.Wait" 
			:loading="nodeInfo.state === NodeState.Navigate" 
			@click="mapStore.mouseAction = mapStore.mouseAction === MouseAction.Navigate ? MouseAction.Control : MouseAction.Navigate"
		> 
			{{ mapStore.mouseAction === MouseAction.Navigate ? '取消' : '导航'}}
		</v-btn>
		<v-btn class="ma-2 bg-primary" size="large" :disabled="nodeInfo.state !== NodeState.Wait" 
			:loading="nodeInfo.state === NodeState.Manual_Init_Pose"
			@click="mapStore.mouseAction = mapStore.mouseAction === MouseAction.SetPosition ? MouseAction.Control : MouseAction.SetPosition"
		> 
			{{ mapStore.mouseAction === MouseAction.SetPosition ? '取消' : '手动设置位置'}}
		</v-btn>
		<v-btn class="ma-2 bg-primary" size="large" :disabled="disabled" 
			:loading="nodeInfo.state === NodeState.Auto_Init_Pose" 
			@click="ctrl.autoInitPose()"
		> 
			自动设置位置 
		</v-btn>
		<v-btn class="ma-2 bg-primary" size="large" :disabled="disabled" 
			:loading="nodeInfo.state === NodeState.Save_Map" 
			@click="ctrl.saveMap()"
		> 
			保存地图 
		</v-btn>
		<v-btn class="ma-2 bg-primary" size="large" :disabled="disabled" 
			:loading="nodeInfo.state === NodeState.Clear_Map" 
			@click="ctrl.clearMap()"
		> 
			清除地图 
		</v-btn>
	</div>
</template>

<script setup lang="ts">
import { useROSStore, NodeState } from '@/stores/ros';
import { useMapStore, MouseAction } from '@/stores/map';
import { ctrl } from '@/ros';

const rosStore = useROSStore();
const mapStore = useMapStore();
const { ctrlMode, nodeInfo } = storeToRefs(rosStore);

const disabled = computed(() => 
	nodeInfo.value.state !== NodeState.Wait || 
	mapStore.mouseAction !== MouseAction.Control
);



</script>