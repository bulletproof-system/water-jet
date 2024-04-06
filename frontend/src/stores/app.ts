// Utilities
import { defineStore } from 'pinia'

export enum ROSState {
  Connected,
  Disconnected,
  Error,
  Connecting
}

export const useAppStore = defineStore('app', {
  state: () => ({
    allowNavigation: false,
    autoControl: true
  }),
  actions: {
    setAllowNavigation(allowNavigation: boolean) {
      this.allowNavigation = allowNavigation
    }
  }
})
