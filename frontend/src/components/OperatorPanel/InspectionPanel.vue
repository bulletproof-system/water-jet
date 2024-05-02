<template>
	<div class="d-flex flex-column ">
		<v-btn class="ma-2 bg-primary" size="large" :disabled="disabled" 
			@click="nodeInfo.state === NodeState.Inspect ? rosStore.cancel() : ctrl.inspect()"
		> 
			{{ nodeInfo.state === NodeState.Inspect ? '停止' : '开始巡检' }}
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
	!(nodeInfo.value.state === NodeState.Wait || nodeInfo.value.state === NodeState.Inspect) || 
	mapStore.mouseAction !== MouseAction.Control
);


</script>