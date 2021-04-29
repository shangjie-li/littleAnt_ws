# SHANGJIELOG

## 2021.04.28
 - 相关坐标系包括
    - global系xyz：东北天
    - gps系xyz：前左上
    - base系xyz：前左上
    - sensor系xyz：前左上
 - 标定坐标系相对位姿
    - 配置gps定位点，使其在水平面内位于车辆轮廓中心。
    - 将激光雷达安装在车辆轮廓中心的正上方。
    - 标定sensor系x轴相对base系x轴的夹角`phi_sensor2base_`。
    - 标定base系x轴相对gps系x轴的夹角`phi_base2gps_`。
    - 将实时定位获得的yaw值作为gps系x轴相对global系x轴的夹角`phi_gps2global_`。
      ```
      # 2021.04.28标定结果
      phi_sensor2base_ = 0.03rad
      phi_base2gps_ = 0.02rad
      ```
 - 测试远程紧急制动
    - 经过测试，50m范围内可有效制动。

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
      

