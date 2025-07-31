# rmcs_dart_guidance
RoboMaster Alliance Team Dart-Launcher Guidance Module

### 前言
（Update-2025.06.23）

当前主分支已废弃，原方法过于简单粗暴，但是对内录、场下分析、传感器融合方面非常的不友好，并且赛时临场改写了一些东西导致控制和视觉尚未解耦，与其清理不如日后重新升级

（Update-2025.06.03）

**2006电机稳定性欠佳，请不要用在关键部位**

视觉设备：hikcamera-CS016-10UC，Livox-Mid-70

### 依赖
运行于南京理工大学Alliance的无下位机方案RMCS（https://github.com/Alliance-Algorithm/RMCS）

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
