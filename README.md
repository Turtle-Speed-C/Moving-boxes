# Box Pick and Place System - README

## 项目概述

这是一个基于ROS的机器人抓放箱任务系统,用于自动化完成箱体的识别、抓取、搬运和放置任务。系统集成了视觉识别、机械臂控制、升降台控制、导航等多个功能模块。

## 核心功能

### 1. 任务流程
系统按以下8个阶段执行完整的抓放任务:

- **阶段1: 初始化** - 升降台复位、状态重置、导航到抓取点
- **阶段2: AprilTag识别** - 视觉定位箱体位置,获取偏移量
- **阶段3: 条码扫描** - 扫描箱体条码,等待MES系统返回目标库位
- **阶段5: 抓取箱体** - 根据AprilTag偏移量调整位置并抓取
- **阶段6: 搬运箱体** - 启动掉箱检测,导航到放置位置
- **阶段7: 放置箱体** - 根据库位号调整高度并放置
- **阶段8: 任务完成** - 复位机械臂和升降台,准备下一次任务

### 2. 关键特性

#### 智能检测系统
- **掉箱检测**: 实时监控电机电流,检测箱体掉落并紧急停止
- **重量检测**: 根据电机电流判断箱体重量等级(轻/中/重)
- **自适应高度调整**: 根据箱体重量自动微调放置高度

#### 多货架支持
- **A货架**: 通用配置,适用于A1-A6库位
- **B货架左列**: 专用配置,适用于B1/B3/B5库位
- **B货架右列**: 专用配置,适用于B2/B4/B6库位
- 每个货架支持3层放置,各层高度和推进距离可独立配置

#### 配置热重载
- 支持任务执行过程中动态重新加载配置文件
- 每次任务开始前自动更新参数,无需重启程序
- 便于现场调试和参数优化

## 系统架构

### 依赖模块
```
- ROS (Robot Operating System)
- WooshApi (AMR导航控制)
- LiftControlClient (升降台控制)
- KuavoRobotArm (机械臂SDK)
- RobotSDK (机器人策略控制)
```

### ROS话题订阅
| 话题名称 | 消息类型 | 功能说明 |
|---------|---------|---------|
| `/position1` | Pose2D | MES返回的目标位置坐标 |
| `/position_location` | String | MES返回的目标库位号 |
| `/barcode_detection_result` | String | 条码扫描结果 |
| `/sensor_data_motor/motor_cur` | Float64MultiArray | 电机电流数据(掉箱/重量检测) |

## 配置文件说明

配置文件 `robot_config_new.yaml` 包含以下主要配置项:

### 机械臂动作配置 (`arm_actions`)
定义不同高度层级的抓取和放置动作轨迹:
- `height_1`: 抓取动作(两阶段)
- `height_2`: 第一层放置动作
- `height_3`: 第二层放置动作
- `height_4`: 第三层放置动作

### 机器人参数配置 (`robot_params`)

#### 基础任务参数
```yaml
task_repeat: 10000              # 任务循环次数
catched_leave_height: 0.23     # 抓取后抬升高度(m)
step_speed: 0.7                # 步进速度(m/s)
step_back_distance: 0.69       # 抓取后后退距离(m)
```

#### 关键位置坐标 (`positions`)
- `catch_point`: 抓取点坐标
- `place_left`: 左侧放置列坐标
- `place_right`: 右侧放置列坐标

#### 条码扫描参数 (`barcode_scan`)
控制扫码阶段的移动步骤和升降台高度

#### AprilTag扫描参数 (`apriltag_scan`)
控制视觉识别阶段的移动步骤

#### 抓取参数 (`catching`)
```yaml
height: 0.24                   # 抓取点升降高度
catch_height_last: 0.21        # 抓取最终高度
x_distance: 0.95               # 抓取时距箱体水平间距
travel_height: 0.28            # 携箱移动高度
right_adjust_distance: 0.19    # 抓取右移调整距离
use_fixed_advance: false       # 是否使用固定前进距离
fixed_advance_distance: 0.12   # 固定前进距离
```

#### 放置参数 (`placements`)
支持通用配置和货架专用配置:
- 通用层级: `first_layer`, `second_layer`, `third_layer`
- A货架专用: `rack_A`
- B货架左列: `rack_B_left` (B1/B3/B5)
- B货架右列: `rack_B_right` (B2/B4/B6)

每层配置包含:
- `target_height`: 放置高度(m)
- `ahead_distance`: 前进距离(m)
- `extra_wait_time`: 额外等待时间(s)

#### 掉箱检测参数 (`drop_detection`)
```yaml
threshold_12: 0.2              # 左臂俯仰关节电流阈值(A)
threshold_13: 3.3              # 左臂横滚关节电流阈值(A)
check_interval: 0.1            # 检测间隔(s)
consecutive_counts: 10         # 连续判定次数
debug_mode: true               # 调试模式开关
```

#### 重量检测参数 (`weight_detection`)
```yaml
enable: true                   # 启用重量检测
light_threshold: 9.5           # 轻箱电流阈值(A)
heavy_threshold: 10.75         # 重箱电流阈值(A)
light_height_offset: -0.04     # 轻箱高度偏移(m)
medium_height_offset: -0.03    # 中箱高度偏移(m)
heavy_height_offset: 0.0       # 重箱高度偏移(m)
```
