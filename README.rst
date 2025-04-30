nmea_navsat_driver_R1
===============

基于nmea_navsat_driver

添加了双天线定位输出的功能适配

1.依赖项：
python-serial或python3-serial 

    $ pip install pyserial
或

    $ pip3 install pyserial

如果没有pip或pip3，请先安装：


    $ sudo apt install python3-pip

或


    $ sudo apt install python-pip

2.下载该功能包到本地目录，例如：catkin_ws/src/

修改端口号：
  
在目录launch/nmea_serial_driver.launch中
 
   <arg name="port" default="/dev/ttyACM3" />

   <arg name="baud" default="115200" />

需要将port的值改为R1在主机实际的端口号
baud改为115200

编译：

    catkin_make

使用：

     roslaunch nmea_navsat_driver nmea_serial_driver.launch

API
---

发布的Topic中：

/fix 

作用：RTK实时定位信息

格式：sensor_msgs/NavSatFix



/vel

作用：移动速度

格式：geometry_msgs/TwistStamped

RTK设备的速度输出。仅当设备输出有效速度信息时发布。驱动程序不会仅根据位置定位来计算速度。需要RTK设备输出NEMA中的GPVTG语句


/heading

作用：航向

格式：geometry_msgs/QuaternionStamped

RTK设备的实时航向，通过双天线测得，需要RTK设备输出NEMA中的GPHDT语句



/time_reference

作用：时间引用

格式：sensor_msgs/TimeReference

RTK设备中读取卫星的时间，作为该TimeReference
