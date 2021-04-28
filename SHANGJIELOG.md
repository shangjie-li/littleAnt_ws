# SHANGJIELOG

## 2021.04.28
 - 梳理坐标系
    - 大地坐标系xyz：东北天
    - 车体（GPS）坐标系xyz：前左上
    - 激光雷达坐标系xyz：前左上

## 2021.04.27
 - 编译littleAnt_ws（不包括av_console）
    - 添加`serial`依赖
      ```
      sudo apt-get install ros-kinetic-serial
      ```
    - 添加`geodesy`依赖
      ```
      sudo apt-get install ros-kinetic-geodesy
      ```
    - 安装`tinyxml2`解析器
      ```
      git clone https://github.com/leethomason/tinyxml2.git
      cd tinyxml2 && sudo make install
      ```

## 2021.04.26
 - 搭建控制单元和检测单元局域网
    - 参考https://blog.csdn.net/qq_37719268/article/details/79019044
    - 设置静态IP
      ```
      # 控制单元：IP地址192.168.2.11，掩码255.255.255.0，勾选Require IPv4
      # 检测单元：IP地址192.168.2.12，掩码255.255.255.0，勾选Require IPv4
      ```
    - 修改hosts文件
      ```
      # 在控制单元中
      sudo gedit /etc/hosts
      # 添加
      192.168.2.11 controller
      192.168.2.12 detector
      
      # 在检测单元中
      sudo gedit /etc/hosts
      # 添加
      192.168.2.11 controller
      192.168.2.12 detector
      ```
    - 修改.bashrc文件
      ```
      # 在控制单元中
      sudo gedit ~/.bashrc
      # 添加
      export ROS_HOSTNAME=controller
      export ROS_MASTER_URI=http://192.168.2.11:11311
      
      # 在检测单元中
      sudo gedit ~/.bashrc
      # 添加
      export ROS_HOSTNAME=detector
      export ROS_MASTER_URI=http://192.168.2.11:11311
      ```
    - 重启计算机，在控制单元中运行`roscore`并接收话题，在检测单元中运行检测算法。
      

