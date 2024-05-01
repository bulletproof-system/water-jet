<template>
	<v-app-bar height="60">
		<v-app-bar-title>Water Jet</v-app-bar-title>
		<v-slide-group v-model="ctrlMode" 
			selected-class="bg-success">
			<v-slide-group-item v-for="mode in modes" :key="mode.value">
				<v-btn :class="['rounded-lg', 'ma-4', `bg-${ctrlMode === mode.value ? mode.color : 'grey'}`, { 'cursor-wait': disable,  }]" 
					width="150" height="45" :value="mode.value"
					:loading="ctrlMode === mode.value && disable"
					:disabled="disable"
					@click="changeMode(mode.value, ctrlMode)" :active="ctrlMode === mode.value">
					{{ mode.name }}
				</v-btn>
			</v-slide-group-item>
		</v-slide-group>
		<!-- <v-spacer></v-spacer> -->
		<v-btn icon="mdi-bell-outline" class="ma-4"></v-btn>
		<v-btn icon="mdi-dots-vertical" class="ma-4"></v-btn>
	</v-app-bar>
</template>

<script setup lang="ts">
import { useROSStore, CtrlMode } from '@/stores/ros';
import { useTheme } from 'vuetify/lib/framework.mjs';
import { ctrl, ros } from '@/ros';
import { useAppStore } from '@/stores/app';

const appStore = useAppStore();
const rosStore = useROSStore();
const ctrlMode = ref<CtrlMode>(rosStore.ctrlMode === CtrlMode.Init ? CtrlMode.Pending : rosStore.ctrlMode);
const modes = [
	{
		name: "等待",
		value: CtrlMode.Pending,
		color: 'blue',
		theme: 'dark',
	},
	{
		name: "自动建图",
		value: CtrlMode.Auto_Map,
		color: 'orange',
		theme: 'work-mode'
	},
	{
		name: "手动建图",
		value: CtrlMode.Manual_Map,
		color: 'orange',
		theme: 'work-mode'
	},
	{
		name: "缺水巡检",
		value: CtrlMode.Inspection,
		color: 'orange',
		theme: 'work-mode'
	},
	{
		name: "指定位置浇水",
		value: CtrlMode.Target,
		color: 'orange',
		theme: 'work-mode'
	},
	{
		name: "巡检自动浇水",
		value: CtrlMode.Auto_Water,
		color: 'orange',
		theme: 'work-mode'
	}
]
const disable = computed(() => rosStore.ctrlMode !== ctrlMode.value)
let theme = useTheme();

// 仅改变当前页面状态即 ctrlMode, rosStore.ctrlMode 由订阅的 info 信息更改
function changeMode(newMode: CtrlMode, oldMode: CtrlMode) {
	if (newMode === oldMode) {
		return;
	}
	ctrlMode.value = newMode;
	// 切换全局模式
	ctrl.changeMode(newMode).then(() => {
		// 模式切换成功后启动节点功能
		ctrl.start();
	}, () => {
		ctrlMode.value = rosStore.ctrlMode
	});
	if (appStore.debug) {
		setTimeout(() => {
			rosStore.setCtrlMode(newMode)
		}, 1000)
	}
		
}

watch(ctrlMode, (newMode, oldMode) => {
	console.log(newMode)
	theme.global.name.value = modes.find(mode => mode.value === newMode)?.theme || 'dark';
}, { immediate: true })

</script>