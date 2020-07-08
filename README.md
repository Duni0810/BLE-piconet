# BLE-piconet
Realize one master and many followers


# 一主三从
该工程实现CC2540一主三从的功能，主机会自动搜素附近广播，连接指定的MAC设备进行自动连接。
连接成功之后，从机会每隔2秒发送一次RSSI数据给主机（通过char4 notice）。

主机从机的通讯通过char6 char7 实现，char6为主机到从机的数据通道（19字节），char7为从机到主机的数据通道（20字节）。
