// Utilities
import { defineStore } from 'pinia'

export enum ROSState {
  Connected,
  Disconnected,
  Error,
  Connecting
}

export const useROSStore = defineStore('ros', {
  state: () => ({
    state: ROSState.Disconnected,
	  retry: 0
  }),
  actions: {
    setRosState(state: ROSState) {
      this.state = state
    },
    resetRetry() {
      this.retry = 0
    }
  }
})
