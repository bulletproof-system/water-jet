<template>
	<div class="d-flex flex-column justify-center align-center">
		<v-progress-circular color="primary" indeterminate ></v-progress-circular>
		<div>
			正在初始化{{ cdots }}
		</div>
	</div>
</template>

<script setup lang="ts">

const cdots = ref('');
const interval = setInterval(() => {
	cdots.value += '.';
	if (cdots.value.length > 3) {
		cdots.value = '';
	}
}, 500);
onBeforeUnmount(() => {
	clearInterval(interval);
})

</script>