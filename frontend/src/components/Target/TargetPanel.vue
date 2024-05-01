<template>
	<div class="d-flex flex-column ">
		<v-btn class="ma-2 bg-primary" size="large" :disabled="disabled" 
			@click="nodeInfo.state === NodeState.Target ? rosStore.cancel() : ctrl.target()"
		> 
			{{ nodeInfo.state === NodeState.Target ? '停止' : '开始' }}
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
	!(nodeInfo.value.state === NodeState.Wait || nodeInfo.value.state === NodeState.Target) || 
	mapStore.mouseAction !== MouseAction.Control
);


</script>