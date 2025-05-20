# rmcs_dart_guidance
RoboMaster Alliance Team Dart-Launcher Guidance Module

设备：hikcamera-CS016-10UC，Livox-Mid-70

### 1.依赖
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