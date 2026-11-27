# PY32F002B 四旋翼飞控与遥控器

该工程基于企业提供的参考固件，用于小型四旋翼的软件链路分析与实机联调。下方区分参考工程和个人工作，原有许可证及源码版权声明保留在各自位置。

| 工程 | Keil 工程入口 | 代码入口 |
| --- | --- | --- |
| 飞控板 | `flight-controller/Projects/MDK-ARM/Project.uvprojx` | `flight-controller/Projects/Src/main.c` |
| 遥控器 | `transmitter/MDK-Project/Project.uvprojx` | `transmitter/User/main.c` |

## 个人实践

- 使用参考硬件及固件进行基础起飞与遥控联调，阅读无线接收、IMU 采样、姿态估计、双环控制及 PWM 输出链路。
- 从源码定位接收端帧头校验与失联计数的顺序问题；提出改进方向并设计无包、连续坏帧、恢复有效帧三个验证场景。仓库中的参考固件未包含这个改动。

## 工程配置

原工程面向 PY32F002Bx5，配置使用 Keil MDK、Puya.PY32F0xx_DFP 1.2.1 和 Arm Compiler 5.06。提交目录仅含源码和构建所需的工程配置，省略编译产物、调试日志和本地编辑器配置。
