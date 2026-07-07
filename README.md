# Factory AMR — ROS 2 Autonomous Mobile Robot

Depoda yük taşıyan, engellerden kaçınan ve el hareketleriyle manuel kontrol edilebilen otonom mobil robot (AMR) simülasyonu. Gazebo Fortress üzerinde diferansiyel sürüşlü bir platform; SLAM ile harita çıkarıyor, Nav2 ile navigasyon yapıyor, BehaviorTree.CPP ile "yük al → taşı → bırak" görevini otonom yürütüyor ve MediaPipe tabanlı el hareketi tanıma ile de sürülebiliyor.

<p align="center">
  <a href="https://www.youtube.com/watch?v=XV6nqqHECoE">
    <img src="https://img.youtube.com/vi/XV6nqqHECoE/maxresdefault.jpg" width="90%" alt="Proje demo videosu — YouTube'da izlemek için tıkla"/>
  </a>
</p>

## Demo

▶️ [Proje demo videosu — YouTube](https://www.youtube.com/watch?v=XV6nqqHECoE)

## Kullanılan Teknolojiler

![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-FF6600?style=for-the-badge&logo=gazebo&logoColor=white)
![Nav2](https://img.shields.io/badge/Nav2-Navigation-blue?style=for-the-badge)
![SLAM Toolbox](https://img.shields.io/badge/SLAM-Toolbox-9cf?style=for-the-badge)
![BehaviorTree.CPP](https://img.shields.io/badge/BehaviorTree.CPP-v3-brightgreen?style=for-the-badge)
![MediaPipe](https://img.shields.io/badge/MediaPipe-Hands-00A98F?style=for-the-badge&logo=google&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)
![C++](https://img.shields.io/badge/C++-17-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
![ros2_control](https://img.shields.io/badge/ros2__control-diff__drive-lightgrey?style=for-the-badge)

## Özellikler

- **URDF/Xacro robot modeli** — diferansiyel tahrikli şasi, kaldırma (lift) mekanizması, IMU, RGB-D kamera ve 2D LiDAR
- **Gazebo Fortress simülasyonu** — `ros2_control` ile diff-drive ve effort-controlled lift joint
- **SLAM (slam_toolbox)** ile harita oluşturma, **Nav2** ile lokalizasyon ve yol planlama
- **BehaviorTree.CPP** tabanlı görev yöneticisi: `NavigateToPose → LiftControl (yukarı) → NavigateToPose → LiftControl (aşağı) → Wait`
- **MediaPipe Hands** ile gerçek zamanlı el hareketi tanıma → robotu ileri/geri/dön ve lift yukarı/aşağı olarak sürme

## Paket Yapısı

| Paket | Açıklama |
|---|---|
| `robot_description` | URDF/Xacro robot modeli, Gazebo entegrasyonu, sensör tanımları |
| `robot_bringup` | Tüm sistemi (Gazebo + robot_state_publisher + ros2_control + RViz) tek launch ile ayağa kaldırır |
| `robot_navigation` | SLAM haritalama ve Nav2 navigasyon launch/parametreleri |
| `robot_mission_control` | BehaviorTree.CPP ile otonom görev yürütücü (NavigateToPose, LiftControl, WaitSeconds action node'ları) |
| `robot_gesture_control` | MediaPipe tabanlı el hareketi tanıma → `cmd_vel` / lift komutu |

## Kurulum

```bash
# Bağımlılıklar: ROS 2 Humble, Gazebo Fortress, Nav2, slam_toolbox, BehaviorTree.CPP v3, MediaPipe
sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-behaviortree-cpp-v3
pip install mediapipe opencv-python

cd ~/Desktop/amr_ws
colcon build --symlink-install
source install/setup.bash
```

## Çalıştırma


**1. Haritalama (SLAM)** ile ortamı keşfet ve haritayı kaydet, ardından **Nav2** ile navigasyonu başlat

```bash
ros2 launch robot_navigation mapping.launch.py - rviz2 
ros2 launch robot_navigation nav2.launch.py
```

<p align="center">
  <img src="readme/navigation.png" width="90%" alt="RViz'de Nav2 costmap ve robot navigasyonu"/>
</p>

**2. Otonom görev** — BehaviorTree ile A noktasından yük al, B noktasına taşı, bırak

```bash
ros2 launch robot_mission_control mission.launch.py
```

<p align="center">
  <img src="readme/bt-liftup.png" width="90%" alt="BehaviorTree görev yürütme — lift kaldırma ve NavigateToPose"/>
</p>

**3. El hareketiyle manuel kontrol**

```bash
ros2 run robot_gesture_control hands_control_node
```

<p align="center">
  <img src="readme/gestureNode.png" width="90%" alt="MediaPipe el hareketi tanıma ile robot kontrolü"/>
</p>

## Lisans

Apache-2.0
