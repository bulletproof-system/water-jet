<template>
	<v-toolbar :collapse="!appStore.showPotList" absolute :style="{ width: config.toolbar_width, 'z-index': 1000 }">
		<v-btn icon @click="appStore.showPotList = !appStore.showPotList" class="ma-n0">
			<v-icon>{{ config.prepend_icon }}</v-icon>
		</v-btn>
		<v-toolbar-title>花盆信息</v-toolbar-title>
	</v-toolbar>
	<v-navigation-drawer :width="config.drawer_width" class="pot-list">
		<div ref="potListRef" style="overflow: hidden scroll; height: inherit;">
			<v-expansion-panels ref="panelsRef" v-show="appStore.showPotList" v-model="appStore.selectedPot">
				<PotInfo v-for="[id, pot] in pots" :ref="(el) => potRefs[id] = el" :key="id" :pot="pot" />
			</v-expansion-panels>
		</div>
	</v-navigation-drawer>
</template>

<script setup lang="ts">
import { pots } from '@/ros';
import { useAppStore } from '@/stores/app';
import { useGoTo } from 'vuetify';

const appStore = useAppStore();
const goTo = useGoTo()
const potListRef = ref(null)
const panelsRef = ref(null)
const potRefs = ref({})

const config = computed(() => {
	if (appStore.showPotList) {
		return {
			drawer_width: '500',
			toolbar_width: '500px',
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

onMounted(() => {
	appStore.setOpenPot(changeSelectedPot)
})
onUnmounted(() => {
	appStore.setOpenPot(() => console.log('unmounted'))
})

// 展开花盆列表并将滚动到选中项
function changeSelectedPot(id: string) {
	if (!pots.value.has(id)) return;
	appStore.setShowPotList(true);
	appStore.setSelectedPot(id);
	setTimeout(() => { // 列表未展开时需要延迟一段时间后进行滚动
		goTo(potRefs.value[id].$el, {
			container: potListRef.value,
			duration: 1000,
			easing: 'easeOutCubic'
		})
	}, 300)
}

</script>

<style scoped>
.pot-list {
	padding-top: 64px;
	overflow: hidden;
	transition: width 0.2s normal;
}
</style>