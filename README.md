# rmcs_dart_guidance
RoboMaster Alliance Team Dart-Launcher Guidance Module

### 0.前言（Update-2025.06.03）
这版镖架制导目前的功能有：
* 全自动
* 视觉对位
* 单发校准（用于解决多发散布大，镖体一致性差的问题，使其能使用同一张射表）
* 前哨基地即时转火（无需预设，引导灯哪里亮了打哪里）
* 点云录制（可以场下离线处理出距离信息）

未实现的功能：
* 即时测距（当场处理出距离信息，查射表进行参数设置，或仅使用镖架导就可具备击打随机固定靶的能力）
  * 纯雷达测距：场地建筑的反射率其实很低，有限的时间内收到的点云细节不足，有时候很难收敛，当前有个demo做到了精度±10cm
  * 相机雷达融合测距：对标定的要求很高
* 视觉跟踪反馈（用于即时修正yaw方向偏差的问题，赋予镖架一定的智能修正能力）

RM25，第一年打RM止步于分区赛，我们确实有一台上限很高的镖架，在家测试的26m单发散布基本都在一个装甲板左右，使用的是4摩擦轮方案。但是在场上，因为天气原因，比平时测试更低的温度和更高的湿度，摩擦轮镖架受到的影响非常大，第一天天气正常打到引导灯附近，第二天阴雨天参数没变直接快打到的基地底座，预先测好的参数通通不能用。

在家测试保护得太好的镖体，在场上的损坏率远高于预期，稳定性大大下降。以及最后一天出现的yaw轴丝杆电机2006异常导致对位失败无法发射，同南航在采访时所说，**2006电机稳定性欠佳，请不要用在关键部位**

无控镖打得稳要看机械，打得准要看算法。后续会继续突破镖架制导，有新的想法和新的实现会在README更新。目前这些东西写的还是非常混乱，很多电控的东西也混在这里（电控视觉都是本人），需要解耦、整理和优化，把最核心的东西整理好方便移植

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
