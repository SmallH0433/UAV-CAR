# WHEELTEC G60 GPS udev 规则：固定别名 /dev/wheeltec_gps
# 注意：厂商原始脚本只有下面第一条 CP2102 规则，但 G60 实物是 CH9102F
# （见《WHEELTEC_G60模块用户手册》，出厂串口序列号 0005），故追加后两条 CH9102 规则。

#CP2102 串口号0005 设置别名为wheeltec_gps
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0005", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_gps"' >/etc/udev/rules.d/wheeltec_gps.rules
#CH9102，同时系统安装了对应驱动 串口号0005 设置别名为wheeltec_gps
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0005", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_gps"' >/etc/udev/rules.d/wheeltec_gps2.rules
#CH9102，同时系统没有安装对应驱动 串口号0005 设置别名为wheeltec_gps
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0005", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_gps"' >/etc/udev/rules.d/wheeltec_gps3.rules

service udev reload
sleep 2
service udev restart
