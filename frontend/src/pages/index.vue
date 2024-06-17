<template>
  <v-layout class="rounded rounded-md app" >
    <AppBar />
      <!-- <v-list style="padding-left: 10px;">
        <v-switch v-model="appStore.allowNavigation" label="启用导航" color="primary"></v-switch>
        <v-switch v-model="appStore.autoControl" label="自动跟随" color="primary"></v-switch>
        <v-switch v-model="appStore.allowObjectRecognition" label="花盆识别" color="primary"></v-switch>
      </v-list> -->

    <v-main class="d-flex align-center justify-center">
      <v-layout  class="rounded rounded-md" full-height>
        <PotList /> 
        <OperatorPanel />
        <v-main>
          <Map />
        </v-main>
      </v-layout>
    </v-main>

  </v-layout>
</template>

<script lang="ts" setup>
import { useAppStore } from '@/stores/app';

definePage({
  // redirect: '/home'
})

const appStore = useAppStore()
const id = ref('')

</script>

<style lang="scss" scoped>
@import '@/styles/variables.scss';

.app {
  width: $app-width;
  height: $app-height;
}

.main {
  height: calc(100% - $app-header-height);
  width: 100%;
}

.footer {
  height: $footer-height;
}

// .footer {
//   height: $footer-height;
// }
</style>

