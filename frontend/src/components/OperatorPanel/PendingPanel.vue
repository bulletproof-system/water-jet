<template>
	<div class="d-flex flex-column ">
		<v-btn class="ma-2 bg-primary" size="large" :disabled="nodeInfo.state !== NodeState.Wait && nodeInfo.state !== NodeState.Navigate || (mouseAction !== MouseAction.Control && mouseAction !== MouseAction.Navigate)" 
			@click="handleNavigateClick()"
		> 
			{{ nodeInfo.state === NodeState.Navigate ? '取消' : mouseAction === MouseAction.Navigate ? '取消' : '导航' }}
		</v-btn>
		<v-btn class="ma-2 bg-primary" size="large" :disabled="nodeInfo.state !== NodeState.Wait || (mouseAction !== MouseAction.Control && mouseAction !== MouseAction.SetPosition)" 
			:loading="nodeInfo.state === NodeState.Manual_Init_Pose"
			@click="mouseAction = mouseAction === MouseAction.SetPosition ? MouseAction.Control : MouseAction.SetPosition"
		> 
			{{ mouseAction === MouseAction.SetPosition ? '取消' : '手动设置位置'}}
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
import { ctrl, ros } from '@/ros';

const rosStore = useROSStore();
const mapStore = useMapStore();
const { ctrlMode, nodeInfo } = storeToRefs(rosStore);
const { mouseAction } = storeToRefs(mapStore);

const disabled = computed(() => 
	nodeInfo.value.state !== NodeState.Wait || 
	mouseAction.value !== MouseAction.Control
);

function handleNavigateClick() {
	switch (nodeInfo.value.state) {
		case NodeState.Wait:
			mouseAction.value = mouseAction.value === MouseAction.Navigate ? MouseAction.Control : MouseAction.Navigate
			break;
		case NodeState.Navigate:
			rosStore.cancel();
			break;
		default:
			console.error('unreachable');
			break;
	}
}

function handleSetPositionClick() {
	if (mouseAction.value === MouseAction.SetPosition) {
		mouseAction.value = MouseAction.Control;
	} else {
		mouseAction.value = MouseAction.SetPosition;
	}
}

// 重置鼠标操作为 Control 
onBeforeUnmount(() => {
	mouseAction.value = MouseAction.Control;
})

</script>