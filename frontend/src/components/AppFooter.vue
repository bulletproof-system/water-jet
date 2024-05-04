<template>
  <v-footer height="50" app>
    <v-badge dot location="start" :color="rosState.color" bordered inline></v-badge>
    <v-btn @click="connectROS(true)" variant="text" width="130">
      {{ rosState.text }}
    </v-btn>

    <!-- <component :is="footers[rosStore.ctrlMode]"  class="flex-fill d-flex justify-center align-center align-self-stretch"></component> -->
    <div class="flex-fill d-flex justify-center align-center align-self-stretch">
      <v-row>
        <v-col offset="2" cols="2" class="d-flex justify-start align-center">
          节点状态:&nbsp;
          <v-chip :color="stateInfo[nodeInfo.state].color" label> {{ stateInfo[nodeInfo.state].msg }} </v-chip>
        </v-col>
        <v-col cols="3" class="d-flex justify-start align-center">
          <v-icon v-if="nodeInfo.percentage == 100" :color="iconInfo[nodeInfo.result].color"> {{ iconInfo[nodeInfo.result].icon }} </v-icon>
          <v-progress-circular v-else-if="nodeInfo.percentage == -1" indeterminate/>
          <v-progress-circular v-else :model-value="nodeInfo.percentage" color="primary"/>
          &nbsp;当前任务:&nbsp;
          {{ stateInfo[nodeInfo.state].task }}
        </v-col>
        <v-col cols="4" class="d-flex justify-start align-center">
          反馈:&nbsp;
          {{ nodeInfo.feedback == '' ? '无' : nodeInfo.feedback }}
        </v-col>
      </v-row>
    </div>

    <v-btn :prepend-icon="scramState.icon" min-width="150"
      :color="scramState.color" variant="elevated" @click="toggleScram" 
      :loading="targetScram !== rosStore.scram"
    >
      {{ scramState.text }}
    </v-btn>
  </v-footer>
</template>

<script setup lang="ts">
import { useAppStore } from '@/stores/app';
import { ConnectState, useROSStore, NodeState } from '@/stores/ros';
import { connectROS, ctrl } from '@/ros';
import * as _ from 'lodash';

const appStore = useAppStore();
const rosStore = useROSStore();

const rosState = computed(() => {
  switch (rosStore.connectState) {
    case ConnectState.Connected: {
      return {
        color: 'success',
        icon: 'mdi-check-circle',
        text: 'Connected'
      }
    }
    case ConnectState.Connecting: {
      return {
        color: 'warning',
        icon: 'mdi-progress-clock',
        text: 'Connecting'
      }
    }
    case ConnectState.Disconnected: {
      return {
        color: 'error',
        icon: 'mdi-close-circle',
        text: 'Disconnected'
      }
    }
    case ConnectState.Error: {
      return {
        color: 'error',
        icon: 'mdi-alert-circle',
        text: 'Error'
      }
    }
  }
})

const { ctrlMode, nodeInfo } = storeToRefs(rosStore);

const stateInfo: Record<NodeState, {msg: string, color: string, task: string}> = {
  [NodeState.Stop]: { msg: '停止', color: 'red', task: '无' },
  [NodeState.Wait]: { msg: '等待', color: 'blue', task: '等待指令' },
  [NodeState.Clear_Map]: { msg: '运行中', color: 'green', task: '清除地图' },
  [NodeState.Auto_Init_Pose]: { msg: '运行中', color: 'green', task: '自动设置机器人位置' },
  [NodeState.Manual_Init_Pose]: { msg: '运行中', color: 'green', task: '手动设置机器人位置' },
  [NodeState.Save_Map]: { msg: '运行中', color: 'green', task: '保存地图' },
  [NodeState.Navigate]: { msg: '运行中', color: 'green', task: '导航' },
  [NodeState.Auto_Init_Map]: { msg: '运行中', color: 'green', task: '自动建图' },
  [NodeState.Manual_Init_Map]: { msg: '运行中', color: 'green', task: '手动建图' },
  [NodeState.Inspect]: { msg: '运行中', color: 'green', task: '缺水巡检' },
  [NodeState.Target]: { msg: '运行中', color: 'green', task: '指定位置浇水' },
  [NodeState.Auto_Water]: { msg: '运行中', color: 'green', task: '自动浇水' },
}


const iconInfo: Record<string, { icon: string, color: string}> = {
	'': {icon: 'mdi-check', color: 'green'},
	'success': {icon: 'mdi-check', color: 'green'},
	'fail': {icon: 'mdi-close', color: 'red'},
	'cancel': {icon: 'mdi-cancel', color: 'red'},
	'error': {icon: 'mdi-alert-circle', color: 'red'},
}

const targetScram = ref(rosStore.scram)
const scramState = computed(() => {
  if (targetScram.value) {
    return {
      color: 'info',
      icon: 'mdi-reload',
      text: '恢复'
    }
  } else {
    return {
      color: 'error',
      icon: 'mdi-stop',
      text: '紧急停止'
    }
  }
})
const toggleScram = _.throttle(() => {
  targetScram.value = !targetScram.value
  ctrl.toggleScram(targetScram.value).catch(() => {
    targetScram.value = rosStore.scram
  });
}, 500)



</script>

<style scoped lang="scss">

</style>
