<template>
  <v-expansion-panel :value="props.pot.id.toString()">
    <v-expansion-panel-title v-slot="{ expanded }" class="align-stretch">
      <v-row no-gutters :align="'stretch'" >
        <v-col class="d-flex justify-start align-center"cols="1">
          {{ props.pot.id }}
        </v-col>
        <v-col class="d-flex justify-start align-center" cols="1">
          <v-btn v-show="rosStore.ctrlMode === CtrlMode.Target" :icon="icon" variant="flat" height="22" width="22"
            class="rounded"
            :disabled="rosStore.nodeInfo.state !== NodeState.Wait || !props.pot.active"
            @click.stop="props.pot.setChoose(!props.pot.choose)"
          >
          </v-btn>
        </v-col>
        <v-col class="d-flex justify-center align-center" cols="8">
          <v-fade-transition leave-absolute v-if="!expanded">
            {{ waterTime }}
          </v-fade-transition>
        </v-col>
        <v-col class="d-flex justify-center align-center" cols="2">
          <v-chip :prepend-icon="props.pot.active ? 'mdi-check' : 'mdi-close'"
            :color="props.pot.active ? 'success' : 'error'" size="small" class="mt-n1"
          >
            {{ props.pot.active ? "启用" : "忽略" }}
          </v-chip>
        </v-col>
      </v-row>
    </v-expansion-panel-title>
    <v-expansion-panel-text>
      <v-row>
        <v-col>
          上次浇水时间: {{ waterTime }}
        </v-col>
      </v-row>
      <v-row>
        <v-col v-if="target.show_picture" cols="12" class="d-flex justify-center align-center">
          <v-img v-if="props.pot.picture" :src="props.pot.picture" cover height="200" width="300" @error="(e) => console.log(e)">
            <template v-slot:placeholder>
              <div class="d-flex align-center justify-center fill-height">
                <v-progress-circular
                  color="grey-lighten-4"
                  indeterminate
                ></v-progress-circular>
              </div>
            </template>
          </v-img>

        </v-col>
        <v-col v-else cols="12" class="d-flex justify-center align-center">
          点云
        </v-col>
      </v-row>
      <v-card-actions class="d-flex justify-end align-center">
        <v-btn color="secondary" variant="text" @click="target.show_picture = !target.show_picture">
          切换
        </v-btn>
        <v-btn :loading="switching" :color="props.pot.active ? 'error' : 'success'" variant="text" @click="switchActive">
          {{ props.pot.active ? "忽略" : "启用" }}
        </v-btn>
        <v-btn color="primary" variant="text" @click="mapStore.locatePot(props.pot.id)">
          定位
        </v-btn>
      </v-card-actions>
    </v-expansion-panel-text>

  </v-expansion-panel>
</template>

<script setup lang="ts">
import { Pot } from '@/ros';
import { useROSStore, CtrlMode, NodeState } from '@/stores/ros';
import { useMapStore } from '@/stores/map';

const props = defineProps({
  pot: {
    type: Pot,
    required: true
  }
})

const rosStore = useROSStore();
const mapStore = useMapStore();

const target = ref({
  active: props.pot.active,
  show_picture: true
})

const switching = computed(() => {
  return target.value.active != props.pot.active
})

const waterTime = computed(() => {
  if (props.pot.last_water_date == null)
    return "无记录"
  else
    return dateTimeFormat.format(props.pot.last_water_date)
})

onMounted(() => {
  target.value.active = props.pot.active
  target.value.show_picture = true
})

onUnmounted(() => {
  
})

const icon = computed(() => {
  return props.pot.active ? props.pot.choose ? 'mdi-checkbox-outline' : 'mdi-checkbox-blank-outline' : 'mdi-minus-box-outline';
})

const dateTimeFormat = new Intl.DateTimeFormat('zh-CN', {
  dateStyle: 'full',
  timeStyle: 'medium',
  hour12: false,
  timeZone: 'Asia/Shanghai'
})

function switchActive() {
  target.value.active = !target.value.active
  props.pot.setActive(target.value.active)
}


</script>

<style scoped>

</style>