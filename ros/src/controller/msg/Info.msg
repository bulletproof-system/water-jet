# /controller/msg/Info.msg
# global mode
uint8 INIT=0
uint8 PENDING=1
uint8 AUTO_MAP=2
uint8 MANUAL_MAP=3
uint8 INSPECTION=4
uint8 TARGET=5
uint8 AUTO_WATER=6

Header header
uint8 mode # 全局模式
bool scram # 急停状态
