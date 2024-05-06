<template>
	<div class="d-flex flex-column ">
		<v-btn class="ma-2 bg-primary" size="large" :disabled="disabled" 
			@click="nodeInfo.state === NodeState.Auto_Init_Map ? rosStore.cancel() : ctrl.autoInitMap()"
		> 
			{{ nodeInfo.state === NodeState.Auto_Init_Map ? '停止建图' : '开始建图' }}
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
	!(nodeInfo.value.state === NodeState.Wait || nodeInfo.value.state === NodeState.Auto_Init_Map) || 
	mapStore.mouseAction !== MouseAction.Control
);


</script>