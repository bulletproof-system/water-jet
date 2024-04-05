<template>
  <Map />
</template>

<script lang="ts" setup>
import { ros } from '@/ros';
import * as ROSLIB from 'roslib';
import { OccupancyGridMsg } from '@/ros/msg';

definePage({
  // redirect: '/home'
})

// let listener: ROSLIB.Topic;
// let grid = ref<OccupancyGridMsg>(null);
// onMounted(() => {
//   listener = new ROSLIB.Topic<OccupancyGridMsg>({
//     ros : ros,
//     name : '/map',
//     messageType : 'nav_msgs/OccupancyGrid',
//     throttle_rate: 1000,
//     compression: 'cbor'
//   });
  
//   listener.subscribe(function(message) {
//     // console.log(message)
//     grid.value = message as unknown as OccupancyGridMsg;
//     console.log(grid)
//     // listener.unsubscribe()
//   });
// })

</script>
