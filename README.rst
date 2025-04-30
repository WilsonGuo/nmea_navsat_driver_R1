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



编译：

    catkin_make

使用：

     roslaunch nmea_navsat_driver nmea_serial_driver.launch

API
---

This package has no released Code API.

The ROS API documentation and other information can be found at http://ros.org/wiki/nmea_navsat_driver
