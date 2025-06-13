# rmcs_dart_guidance
RoboMaster Alliance Team Dart-Launcher Guidance Module

### 0.前言（Update-2025.06.03）
**2006电机稳定性欠佳，请不要用在关键部位**

**目标是将飞镖系统做到能稳定成为场上战术的一环而不是随机的惊喜**

视觉设备：hikcamera-CS016-10UC，Livox-Mid-70

### 1.依赖
本项目使用南京理工大学Alliance的无下位机方案RMCS（https://github.com/Alliance-Algorithm/RMCS）

关于RMCS已经有比较完整的部署方法，欢迎使用

* livox_sdk，依据雷达型号选择

```
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd path_to_Livox-SDK/buide
cmake -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE ..
make
sudo make install
```

* PCL

```
sudo apt install libpcl-dev libpcl-ros-dev

sudo apt install ros-humble-pcl-conversions ros-humble-pcl-msgs ros-humble-pcl-ros
```
