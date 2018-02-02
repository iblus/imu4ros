# imu4ros
UART interface The ROS node of the NAVI
------
## Description
...

### 文件说明：
leador_imu_uart -- 导航的采集ROS节点源码
leador_msgs -- 导航的ROS msg源码
listener -- 用于接收导航节点msg的例程

### 编译说明：
1. 使用catkin_make首先编译leador_msgs
2. 使用catkin_make 编译leador_imu_uart
3. 使用catkin_make 编译listener

### 运行说明：
1. 使用chmod分别给**导航**的串口设备文件读写的权限
	sudo chmod 777 /dev/ttyUSB0
	其中"/dev/ttyUSB0"为串口的设备文件名(实际情况中设备名可能有变化)
2. 运行leador_imu_uart 节点
	rosrun leador_imu_uart imu_node /dev/ttyUSB0
	其中"/dev/ttyUSB0"为连接**导航**的串口的设备文件名(实际情况中设备名可能有变化)
3. 运行leador_imu_uart 节点后即可接收导航的msg，具体可参考listener例程.
	rosrun leador_imu_listener listener_node

	
