第一次推送，可能有些没有顾及到的地方，欢迎大家讨论修改

我是供电统一5v
接线
发射端：
esp32 gpio16---mos管 io
mosvin+ mosvin- 接5v电源和地
mos out+接电灯泡，串一个2欧电阻分压
接收端：
nodemcu a0 ————0.1uf电容————LM358out
LM358in ---0.1uf电容--bpw34阴极
bpw34阳极--gnd
下拉电阻10k接a0
