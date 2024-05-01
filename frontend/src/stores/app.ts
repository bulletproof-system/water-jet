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
    debug: true,
    allowNavigation: false,
    autoControl: true,
    allowObjectRecognition: false,
    showPotList: true,
    showOperatorPanel: true,
    selectedPot: '',
    openPot: (id: string) => console.log('unmounted'),
    locatePot: (id: string) => console.log('unmounted'),
  }),
  actions: {
    setAllowNavigation(allowNavigation: boolean) {
      this.allowNavigation = allowNavigation
    },
    setShowPotList(showPotList: boolean) {
      this.showPotList = showPotList
    },
    setShowOperatorPanel(showOperatorPanel: boolean) {
      this.showOperatorPanel = showOperatorPanel
    },
    setSelectedPot(id: string) {
      this.selectedPot = id
    },
    setOpenPot(f: (id: string) => void) {
      this.openPot = f
    }
  }
})
