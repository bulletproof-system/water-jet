<template>
  <v-app class="rounded rounded-md app" >
    <v-app-bar title="Water Jet"></v-app-bar>

    <v-navigation-drawer>
      <v-list style="padding-left: 10px;">
        <v-switch v-model="appStore.allowNavigation" label="启用导航" color="primary"></v-switch>
        <v-switch v-model="appStore.autoControl" label="自动跟随" color="primary"></v-switch>
      </v-list>
    </v-navigation-drawer>

    <v-main class="d-flex align-center justify-center">
      <Map />
    </v-main>

    <AppFooter />
  </v-app>

</template>

<script lang="ts" setup>
import { useAppStore } from '@/stores/app';

definePage({
  // redirect: '/home'
})

const appStore = useAppStore()

</script>

<style lang="scss" scoped>
@import '@/styles/variables.scss';

.app {
  width: $app-width;
  height: $app-height;
}

.footer {
  height: $footer-height;
}
</style>

