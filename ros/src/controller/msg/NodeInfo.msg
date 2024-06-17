# /controller/msg/NodeInfo.msg
# 通用
uint8 Stop=0
uint8 Wait=1

# Pending
uint8 Navigate=2
uint8 Auto_Init_Pos=3
uint8 Manual_Init_Pos=4
uint8 Clear_Map=5
uint8 Save_Map=6

# Auto_Map
uint8 Auto_Init_Map=7

# Manual_Map
uint8 Manual_Init_Map=8

# Inspection
uint8 Inspect=9

# Target
uint8 Target=10

# Auto_Water
uint8 Auto_Water=11

uint8 mode  # 节点模式
uint8 state # 节点状态
