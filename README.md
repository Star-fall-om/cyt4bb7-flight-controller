# CYT4BB7 无人机飞控

从第 21 届全国大学生智能汽车竞赛空地协同项目中提取的 CYT4BB7 Cortex-M7_0 飞控源码。包含姿态与水平位置控制、定高、速度估计和电机输出。源码保留原工程的目录结构，可加入使用 IAR Embedded Workbench、逐飞 CYT4BB7 库和 FreeRTOS 的 M7_0 工程。

## 模块

| 目录 | 功能 |
| --- | --- |
| `code/Algorithm/PID/` | 姿态、角速度、水平位置与速度、定高控制。 |
| `code/IMU/` | IMU963 采集、姿态解算、垂向状态估计和水平速度卡尔曼滤波。 |
| `code/LC302/`、`code/TOF/` | 光流速度估计和测距高度处理。 |
| `code/Motor/` | 四路电机 PWM 输出。 |
| `code/Navigation/` | 把机体系光流速度按偏航角转换到导航坐标系，更新水平位置和速度，并写入飞行侧下行状态。 |
| `code/Config/` | 飞行功能开关和控制参数默认值。参数对应原机架、传感器安装及控制周期。 |
| `code/System/` | M7_0 故障记录、串口中断适配和 TFT 驱动选择。 |
| `code/Utility/VOFA/` | VOFA+ 调参和控制环观测接口。 |
| `code/Vision/`、`code/Camera/` | 飞行侧使用的跨核视觉调试结构和视觉模式定义。 |

## 接入 M7_0 工程

1. 准备匹配硬件的 IAR M7_0 基工程。时钟启动、任务调度、外设驱动、链接脚本和芯片 SDK 由基工程提供。接入现有应用时，还需提供匹配的 `System/usr_system.h`、`System/usr_uart.h` 和 `Vision/Vision.h` 及其实现。逐飞库提供基础 SDK 和驱动，原机使用的 DL1B3、TFT 等驱动按硬件准备。[逐飞 CYT4BB7 开源库](https://gitee.com/seekfree/CYT4BB7_Library)
2. 按相对路径放入所需的 `code/` 模块，将 `code/` 加入 IAR C/C++ Compiler 的附加头文件目录，并将所选模块的 `.c` 文件加入 M7_0 项目。只使用水平速度滤波时，加入 `horizontal_velocity_kf.c` 及其同目录头文件即可。
3. 按下表连接相应模块。使用已有 M7_0 工程时，先核对同名模块的接口版本；将硬件引脚、传感器方向和控制参数配置为本机值。

| 模块 | 接入点和依赖 |
| --- | --- |
| `PID`、`Motor` | 基工程提供飞控状态和时间基准、按控制周期更新目标和反馈，并将计算结果分配到四路输出。核对 PWM 引脚、电机编号及转向。 |
| `IMU963`、`LC302`、`TOF` | 基工程负责传感器初始化顺序、采样任务和外设接口。TOF 高度使用 mm，水平估计的速度和位置分别使用 cm/s 和 cm。 |
| `horizontal_velocity_kf` | 初始化后按实际采样间隔调用 `horizontal_velocity_kf_predict()`。取得光流观测时调用 `horizontal_velocity_kf_update()`。加速度用 cm/s²，速度用 cm/s，采样间隔用 s。 |
| `Navigation` | 调用 `Navigation_UpdateFromFlow()` 更新状态。由基工程提供匹配的 `Vision/Vision.h` 和 `SharedData_Down_t`，再调用 `Navigation_FillSharedData()` 填充导航字段。 |
| `m70_fault` | 由 M7_0 启动流程调用 `M70Fault_BootInit()` 并按需打印记录。故障入口依赖 CMSIS、FreeRTOS 和电机输出接口。使用 IAR 时检查 `__no_init` 段和中断入口定义。 |
| `uart_irq_compat`、`tft180_driver_select` | 串口适配依赖逐飞 SCB 接口。显示选择依赖基工程配置和对应 TFT 驱动。 |
| `vofa_tune` | 依赖 PID、电机、飞控系统和串口接口。调参任务按 10 ms 调用 `VofaTune_Process10ms()`；电机输出路径可通过 `VofaTune_UpdateMotorOutputs()` 提供四路输出。 |

### 双核接口

原工程使用 `0x28020000` 传递 M7_1 到 M7_0 的视觉控制数据，使用 `0x28020100` 传递 M7_0 到 M7_1 的飞行状态。两侧共享控制结构由基工程配套提供。发布的 `vision_shared_debug_t` 是另一路调试快照，原工程地址为 `0x28020400`，结构大小为 124 字节。跨核使用时，两侧需采用一致的结构布局，在发送端写入后执行 DCache clean，在接收端读取前执行 DCache invalidate。

该仓库提供模块源码。生成固件需要适配后的 M7_0 基工程和目标板配置。

## 许可证

飞控源码采用 GPL-3.0。[许可证全文](LICENSE)
