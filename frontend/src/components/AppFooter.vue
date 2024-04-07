<template>
  <v-footer height="40" app>
    <v-badge dot location="start" :color="rosState.color" bordered inline>
    </v-badge>
      <v-btn @click="connectROS">
        {{ rosState.text }}
      </v-btn>
  </v-footer>
</template>

<script setup lang="ts">
import { useAppStore } from '@/stores/app';
import { ROSState, useROSStore } from '@/stores/ros';
import { connectROS } from '@/ros';

const rosStore = useROSStore();

const rosState = computed(() => {
  switch (rosStore.state) {
    case ROSState.Connected: {
      return {
        color: 'success',
        icon: 'mdi-check-circle',
        text: 'Connected'
      }
    }
    case ROSState.Connecting: {
      return {
        color: 'warning',
        icon: 'mdi-progress-clock',
        text: 'Connecting'
      }
    }
    case ROSState.Disconnected: {
      return {
        color: 'error',
        icon: 'mdi-close-circle',
        text: 'Disconnected'
      }
    }
    case ROSState.Error: {
      return {
        color: 'error',
        icon: 'mdi-alert-circle',
        text: 'Error'
      }
    }
  }
})

</script>

<style scoped lang="scss">

</style>
