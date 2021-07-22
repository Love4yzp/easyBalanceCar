# 简易平衡车 基于 HAL库

#### 介绍
 全部代码基于HAL库

#### 软件架构

```
├─Core
│  ├─Inc
│  └─Src
└─Lib
  ├─mpu6050    | 自带卡尔曼滤波处理，对原始数据进行了处理，借用_tm_的库可以实现中断
  ├─oled    | OLED显示，能够滚动显示
  └─print    | 实现printf串口重定向
```

#### 安装教程
本次环境全基于：
- Clion
如要用Keil 则需要通过 STM32CubeMX 进行代码生成，然后在Keil中导入相对应的库文件。

#### 使用说明

#### MPU6050
曾尝试过将国内的代码进行整合，无奈国内嵌入式开发者都没有相应的数据结构·算法素养，导致移植困难，于是找到如下库，进行了整合
- [MPU6050 卡尔曼滤波(Kalman filter)](https://github.com/leech001/MPU6050)
- [tm 通用MPU6050库 INT中断函数](https://github.com/MaJerle/stm32fxxx-hal-libraries)
#### OLED
GitHub 中顺便找的 SSD1306库实现的。
在[Display Library](https://github.com/anothermist/DISPLAYS)中有各种开发环境的库
### END
---
代码暂未进行处理。