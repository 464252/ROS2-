# FishBot CAM驱动

下载：

```
mkdir -p ~/fishbot_ws/src/ && cd ~/fishbot_ws/src/
git clone https://github.com/fishros/fishbot_camera.git
```

构建：

```
cd ~/fishbot_ws/
colcon build
```

运行:

```
source ~/fishbot_ws/install/setup.bash
ros2 run fishbot_camera camera_driver
```


**配置文档**

---

**一、LED亮度调整**

LED的亮度可调整，亮度值范围在0-255之间。

---

**二、图像质量调整**

图像质量可调整，质量值范围在4-63之间。

---

**三、水平镜像**

水平镜像功能可开启或关闭。

- val=0：关闭水平镜像
- val=1：开启水平镜像

---

**四、上下反转**

上下反转功能可开启或关闭。

- val=0：关闭上下反转
- val=1：开启上下反转


---

**五、人脸检测**

人脸检测功能可开启或关闭。

- val=0：关闭人脸检测
- val=1：开启人脸检测

---

**六、图像大小调整**

图像大小可调整，具体大小选项见下表。

**图像大小选项**

| Label | Value |
| --- | --- |
| UXGA(1600x1200) | 13 |
| SXGA(1280x1024) | 12 |
| HD(1280x720) | 11 |
| XGA(1024x768) | 10 |
| SVGA(800x600) | 9 |
| VGA(640x480) | 8 |
| HVGA(480x320) | 7 |
| CIF(400x296) | 6 |
| QVGA(320x240) | 5 |
| 240x240 | 4 |
| HQVGA(240x176) | 3 |
| QCIF(176x144) | 2 |
| QQVGA(160x120) | 1 |
| 96x96 | 0 |


---

**七、特殊效果**

特殊效果可设置，效果自行设置对比。