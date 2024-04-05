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
  })
})
