# arx5_ros
arx5机械臂的SDK使用说明（待定）

## 简介
* arx5_bringup: 整合启动包
* arx5_control: 底层电机接口
* arx5_description: urdf文件
* arx5_driver:  机械臂控制算法
* arx5_teleop:  手柄与键盘控制接口

## 安装
```sh
cd src
git clone https://gitee.com/arx-discover/arx5_ros.git
cd ..
catkin_make
echo "安装结束"
```

## 非moveit的Demo运行
```sh

```

# 开发者

## sudo pip3 install -e .  允许软连接方便调试

## 卸载： sudo pip3 uninstall package_name