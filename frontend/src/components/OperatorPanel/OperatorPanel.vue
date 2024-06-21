<template>
	<v-toolbar :collapse="!appStore.showOperatorPanel" absolute :class="['toolbar', appStore.showOperatorPanel ? '' : 'collapse' ]" :style="{ width: config.toolbar_width }">
		<v-toolbar-title>操作面板</v-toolbar-title>
		<v-spacer v-if="!appStore.showOperatorPanel"></v-spacer>
		<v-btn icon @click="appStore.showOperatorPanel = !appStore.showOperatorPanel" class="ma-n0 ">
			<v-icon>{{ config.prepend_icon }}</v-icon>
		</v-btn>
	</v-toolbar>
	<v-navigation-drawer location="right" :width="config.drawer_width" class="operator-panel">
		<div style="overflow: hidden; height: inherit;">
			<v-fab-transition origin="top center">
				<component :is="panels[rosStore.ctrlMode]"  style="height: inherit;"></component>
			</v-fab-transition>
		</div>
	</v-navigation-drawer>
</template>

<script setup lang="ts">
import { useAppStore } from '@/stores/app';
import { useROSStore, CtrlMode } from '@/stores/ros';

const appStore = useAppStore();
const rosStore = useROSStore();
const id = ref('')

const config = computed(() => {
	if (appStore.showOperatorPanel) {
		return {
			drawer_width: '200',
			toolbar_width: '200px',
			prepend_icon: 'mdi-menu-open'
		}
	} else {
		return {
			drawer_width: '0.1',
			toolbar_width: '70px',
			prepend_icon: 'mdi-menu'
		}
	}
})

const panels = (() => {
  let panels = {}
  for (const key in CtrlMode)
    if (isNaN(key as any))
	panels[CtrlMode[key]] = defineAsyncComponent(() => import(`@/components/OperatorPanel/${key}Panel.vue`))
  return panels
})()

</script>

<style scoped>
.toolbar {
	z-index: 1000;
	right: 0;
}

.collapse {
	border-end-end-radius: 0;
	border-end-start-radius: 24px;
}

.operator-panel {
	padding-top: 64px;
	overflow: hidden;
	transition: width 0.2s normal;
}
</style>