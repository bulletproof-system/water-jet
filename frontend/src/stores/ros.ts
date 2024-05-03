// Utilities
import { defineStore } from 'pinia'

export enum ConnectState {
  Connected,
  Disconnected,
  Error,
  Connecting
}

export enum CtrlMode {
  Init = 0, 
  Pending = 1,
  Auto_Map = 2,
  Manual_Map = 3,
  Inspection = 4,
  Target = 5,
  Auto_Water = 6
}

export enum NodeState {
	Stop = 0,
	Wait,
	Navigate,
	Auto_Init_Pose,
	Manual_Init_Pose,
	Clear_Map,
	Save_Map,
  Auto_Init_Map,
  Manual_Init_Map,
  Inspect,
  Target,
  Auto_Water,
}

export interface NodeInfo {
  state: NodeState,
  task: string,
  feedback: string,
  result: 'success' | 'fail' | 'cancel' | 'error' | '',
  percentage: number,
  cancel: () => void, // 取消当前操作
}

export const useROSStore = defineStore('ros', {
  state: () => ({
    connectState: ConnectState.Disconnected,
	  retry: 0,
    ctrlMode: CtrlMode.Init,
    scram: false,
    nodeInfo: {
      state: NodeState.Stop,
      task: '',
      feedback: '',
      result: '',
      percentage: 100,
      cancel: null,
    } as NodeInfo,
  }),
  actions: {
    setConnectState(connectState: ConnectState) {
      this.connectState = connectState
    },
    resetRetry() {
      this.retry = 0
    },
    setCtrlMode(ctrlMode: CtrlMode) {
      this.ctrlMode = ctrlMode
    },
    setScram(active: boolean) {
      this.scram = active
    },
    setNodeState(state: NodeState) {
      this.nodeInfo.state = state
    },
    cancel() {
      if (!this.nodeInfo.cancel) return;
      this.nodeInfo.cancel();
    }
  }
})
